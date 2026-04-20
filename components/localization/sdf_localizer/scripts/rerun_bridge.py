#!/usr/bin/env python3
"""
rerun_bridge.py — Real-time telemetry bridge for sdf_localizer → Rerun.

Listens on TCP localhost:9877 for binary frames sent by RerunLogger (C++).
Each frame: [4-byte LE length][JSON payload].

Logged entities (Rerun paths):
  world/robot          – Transform3D (pose axes)
  world/lidar          – Points3D    (LIDAR scan in robot frame, transformed)
  world/path/adam      – LineStrips3D (trajectory when Adam ran)
  world/path/pred      – LineStrips3D (trajectory when early-exit/prediction used)
  world/sdf            – DepthImage / Tensor (SDF grid, every N frames)
  metrics/timing/*     – Scalars     (t_update, t_adam, t_cov, t_breakdown)
  metrics/loss/*       – Scalars     (loss_init, final_loss_adam, final_loss_pred, l_bnd, l_obs, l_mot)
  metrics/quality/*    – Scalars     (sdf_mse, innov_norm, cov_xx, cov_tt)
  metrics/pipeline/*   – Scalars     (ws, iters, early_exit flag)
  metrics/debug/pose_table – TextLog (predicted/optimized/innovation/uncertainty)
"""

import socket
import struct
import json
import base64
import math
import traceback
import sys
import time
import threading
import argparse

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

# ──────────────────────────────────────────────────────────────────────────────
HOST = "0.0.0.0"
PORT = 9877
SDF_WALL_BAND_M = 0.08     # cells with |sdf| <= band are considered wall surface
SDF_WALL_MAX_POINTS = 25000
SDF_WALL_ALPHA_MAX = 170    # max transparency (0..255) for wall overlay

# ──────────────────────────────────────────────────────────────────────────────
def decode_b64_floats(b64_str):
    """Decode base64 string → numpy float32 array."""
    if b64_str is None:
        return None
    raw = base64.b64decode(b64_str)
    return np.frombuffer(raw, dtype="<f4")   # little-endian float32


def _scalar_archetype(v):
    """Compatibility wrapper across rerun versions.

    Newer versions expose rr.Scalars([...]); older ones may expose rr.Scalar(...).
    """
    if hasattr(rr, "Scalars"):
        return rr.Scalars([float(v)])
    if hasattr(rr, "Scalar"):
        return rr.Scalar(float(v))
    raise AttributeError("rerun module has no Scalar/Scalars archetype")


def _log_scalar(path, value):
    rr.log(path, _scalar_archetype(value))


def build_blueprint():
    """Define the Rerun UI layout."""
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                name="3D World",
                origin="/",
                contents=[
                    "world/robot/**",
                    "world/lidar",
                    "world/path/**",
                    "world/room/outline",
                    "world/uncertainty/**",
                ],
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(
                    name="Timing (ms)",
                    origin="/metrics/timing",
                ),
                rrb.TimeSeriesView(
                    name="Loss terms",
                    origin="/metrics/loss",
                ),
                rrb.TimeSeriesView(
                    name="Quality",
                    origin="/metrics/quality",
                ),
                rrb.TimeSeriesView(
                    name="Pipeline",
                    origin="/metrics/pipeline",
                ),
                rrb.TextDocumentView(
                    name="Pose Debug Table",
                    origin="/metrics/debug/pose_table_doc",
                ),
            ),
        ),
        collapse_panels=True,
    )


class RerunBridge:
    def __init__(self):
        # Keep trajectory as contiguous segments to avoid false line jumps
        # when mode alternates between Adam and Pred.
        self.path_segments_adam = []   # list[list[[x,y], ...]]
        self.path_segments_pred = []
        self.last_mode_early = None
        self.frame_idx = 0
        self.room_outline_logged = False

    @staticmethod
    def _ellipse_from_cov(cov_xy_2x2: np.ndarray, sigma_scale: float = 2.0, n: int = 64):
        """Return Nx2 ellipse points for covariance contour; None when invalid."""
        if cov_xy_2x2.shape != (2, 2):
            return None
        if not np.isfinite(cov_xy_2x2).all():
            return None
        # Symmetrize and clamp tiny negative eigenvalues from numeric noise.
        cov = 0.5 * (cov_xy_2x2 + cov_xy_2x2.T)
        vals, vecs = np.linalg.eigh(cov)
        vals = np.maximum(vals, 1e-9)
        t = np.linspace(0.0, 2.0 * math.pi, num=n, endpoint=True)
        unit = np.stack([np.cos(t), np.sin(t)], axis=1)
        radii = sigma_scale * np.sqrt(vals)
        return unit @ np.diag(radii) @ vecs.T

    @staticmethod
    def _heading_sector(center_x: float, center_y: float, theta: float, sigma_theta: float,
                        radius: float = 0.8, sigma_scale: float = 2.0, n_arc: int = 28):
        """Build sector polyline (center->arc->center) for heading uncertainty."""
        half_angle = float(sigma_scale * sigma_theta)
        half_angle = min(max(half_angle, 0.0), math.pi)
        if half_angle <= 1e-6:
            return None
        # In this stack theta=0 points to +Y (forward), not +X.
        heading_forward = theta + (0.5 * math.pi)
        arc_angles = np.linspace(heading_forward - half_angle, heading_forward + half_angle, num=n_arc, endpoint=True)
        arc = np.stack([
            center_x + radius * np.cos(arc_angles),
            center_y + radius * np.sin(arc_angles),
            np.full(n_arc, 0.02, dtype=np.float32),
        ], axis=1).astype(np.float32)
        center = np.array([[center_x, center_y, 0.02]], dtype=np.float32)
        return np.vstack([center, arc, center])

    @staticmethod
    def _trim_segments(segments, max_points=5000):
        total = sum(len(seg) for seg in segments)
        while total > max_points and segments:
            total -= len(segments[0])
            segments.pop(0)

    # ── initialise Rerun ──────────────────────────────────────────────────────
    def init_rerun(self, spawn_viewer: bool):
        rr.init("sdf_localizer", spawn=spawn_viewer)
        rr.send_blueprint(build_blueprint())

        # World coordinate axes (static)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

    # ── process one decoded JSON frame ───────────────────────────────────────
    def process(self, d: dict):
        ts_ms = d.get("ts_ms", 0)

        # Rerun API compatibility across versions:
        # - Newer versions expose rr.set_time(..., sequence=..., timestamp=...)
        # - Older versions used rr.set_time_sequence / rr.set_time_nanos
        if hasattr(rr, "set_time"):
            rr.set_time("frame", sequence=self.frame_idx)
            rr.set_time("wall_clock", timestamp=ts_ms / 1000.0)
        else:
            if hasattr(rr, "set_time_sequence"):
                rr.set_time_sequence("frame", self.frame_idx)
            if hasattr(rr, "set_time_nanos"):
                rr.set_time_nanos("wall_clock", ts_ms * 1_000_000)

        self.frame_idx += 1

        x      = d.get("x", 0.0)
        y      = d.get("y", 0.0)
        theta  = d.get("theta", 0.0)
        early  = d.get("early_exit", False)

        # ── 1. Robot pose (Transform + axes) ─────────────────────────────────
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        # 3×3 rotation matrix (in the XY plane, Z up)
        rot = np.array([
            [ cos_t, -sin_t, 0.0],
            [ sin_t,  cos_t, 0.0],
            [  0.0,    0.0,  1.0],
        ], dtype=np.float32)

        rr.log("world/robot",
               rr.Transform3D(
                   translation=[x, y, 0.0],
                   mat3x3=rot,
               ))

        # Draw robot body as a small box
        rr.log("world/robot/body",
               rr.Boxes3D(half_sizes=[[0.25, 0.20, 0.15]],
                          colors=[[50, 200, 50]]))

        # Robot local axes (X right, Y forward, Z up)
        rr.log("world/robot/axes",
               rr.Arrows3D(
                   origins=[[0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0]],
                   vectors=[[0.5, 0.0, 0.0],   # +X right
                            [0.0, 0.5, 0.0],   # +Y forward
                            [0.0, 0.0, 0.25]], # +Z up
                   colors=[[255, 80, 80],
                           [80, 255, 80],
                           [80, 140, 255]]))

        # ── 1.5 Pose uncertainty: position ellipse + heading sector ────────────
        cov_xx = float(d.get("cov_xx", float("nan")))
        cov_xy = float(d.get("cov_xy", 0.0))
        cov_yy = float(d.get("cov_yy", cov_xx))
        cov_tt = float(d.get("cov_tt", float("nan")))

        if np.isfinite(cov_xx) and np.isfinite(cov_yy) and cov_xx >= 0.0 and cov_yy >= 0.0:
            cov2 = np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]], dtype=np.float32)
            ellipse_xy = self._ellipse_from_cov(cov2, sigma_scale=2.0, n=72)
            if ellipse_xy is not None:
                ellipse_world = np.column_stack([
                    ellipse_xy[:, 0] + x,
                    ellipse_xy[:, 1] + y,
                    np.full(ellipse_xy.shape[0], 0.02, dtype=np.float32),
                ]).astype(np.float32)
                rr.log("world/uncertainty/position",
                       rr.LineStrips3D([ellipse_world],
                                       colors=[[255, 120, 0]],
                                       radii=[0.006]))

        if np.isfinite(cov_tt) and cov_tt >= 0.0:
            sigma_theta = math.sqrt(cov_tt)
            # Radius tied to translational sigma so angular fan scales with pose quality.
            sigma_pos = math.sqrt(max(cov_xx, 1e-6)) if np.isfinite(cov_xx) and cov_xx >= 0.0 else 0.1
            sector_radius = float(np.clip(0.6 + 2.0 * sigma_pos, 0.4, 1.5))
            sector = self._heading_sector(x, y, theta, sigma_theta,
                                          radius=sector_radius,
                                          sigma_scale=2.0,
                                          n_arc=32)
            if sector is not None:
                rr.log("world/uncertainty/heading",
                       rr.LineStrips3D([sector],
                                       colors=[[255, 80, 180]],
                                       radii=[0.005]))

        # ── 2. Accumulated path (segmented by contiguous mode) ───────────────
        mode_changed = (self.last_mode_early is None) or (self.last_mode_early != early)
        active_segments = self.path_segments_pred if early else self.path_segments_adam
        if mode_changed or not active_segments:
            active_segments.append([])
        active_segments[-1].append([x, y])
        self.last_mode_early = early

        self._trim_segments(self.path_segments_adam, max_points=5000)
        self._trim_segments(self.path_segments_pred, max_points=5000)

        adam_strips = []
        for seg in self.path_segments_adam:
            if len(seg) >= 2:
                pts = np.array(seg, dtype=np.float32)
                adam_strips.append(np.column_stack([pts, np.zeros(len(pts), dtype=np.float32)]))
        if adam_strips:
            rr.log("world/path/adam",
                   rr.LineStrips3D(adam_strips,
                                   colors=[[255, 210, 60]],
                                   radii=[0.01]))

        pred_strips = []
        for seg in self.path_segments_pred:
            if len(seg) >= 2:
                pts = np.array(seg, dtype=np.float32)
                pred_strips.append(np.column_stack([pts, np.zeros(len(pts), dtype=np.float32)]))
        if pred_strips:
            rr.log("world/path/pred",
                   rr.LineStrips3D(pred_strips,
                                   colors=[[170, 170, 170]],
                                   radii=[0.008]))

        # ── 3. LIDAR point cloud (in world frame) ─────────────────────────────
        n_pts = d.get("n_pts", 0)
        pts_b64 = d.get("pts")
        if pts_b64 and n_pts > 0:
            flat = decode_b64_floats(pts_b64)
            if flat is not None and len(flat) == n_pts * 3:
                pts_robot = flat.reshape(n_pts, 3)   # robot frame [x, y, z]
                # Transform to world frame: p_world = R @ p_robot + t
                pts_world_xy = (rot[:2, :2] @ pts_robot[:, :2].T).T + np.array([x, y])
                pts_world = np.column_stack([pts_world_xy, pts_robot[:, 2]])
                color = [100, 180, 255] if not early else [180, 180, 180]
                rr.log("world/lidar",
                       rr.Points3D(pts_world,
                                   radii=[0.02],
                                   colors=[color]))

        # ── 3.5 Room outline from real polygon (one-time) ────────────────────
        if not self.room_outline_logged:
            poly_n = int(d.get("room_poly_n", 0))
            poly_b64 = d.get("room_poly")
            if poly_b64 and poly_n >= 3:
                flat_poly = decode_b64_floats(poly_b64)
                if flat_poly is not None and len(flat_poly) == poly_n * 2:
                    poly = flat_poly.reshape(poly_n, 2).astype(np.float32)
                    poly3 = np.column_stack([poly, np.full(poly_n, 0.03, dtype=np.float32)])
                    poly3_closed = np.vstack([poly3, poly3[0]])
                    rr.log("world/room/outline",
                           rr.LineStrips3D([poly3_closed],
                                           colors=[[255, 170, 90]],
                                           radii=[0.01]))
                    self.room_outline_logged = True

        # ── 4. Fallback room outline from first SDF metadata only ─────────────
        if not self.room_outline_logged and d.get("sdf") is not None:
            w = int(d.get("sdf_w", 0))
            h = int(d.get("sdf_h", 0))
            ox = float(d.get("sdf_ox", 0.0))
            oy = float(d.get("sdf_oy", 0.0))
            cell = float(d.get("sdf_cell", 0.0))
            if w > 1 and h > 1 and cell > 0.0:
                x0 = ox
                y0 = oy
                x1 = x0 + w * cell
                y1 = y0 + h * cell
                outline = np.array([
                    [x0, y0, 0.03],
                    [x1, y0, 0.03],
                    [x1, y1, 0.03],
                    [x0, y1, 0.03],
                    [x0, y0, 0.03],
                ], dtype=np.float32)
                rr.log("world/room/outline",
                       rr.LineStrips3D([outline],
                                       colors=[[255, 170, 90]],
                                       radii=[0.01]))
                self.room_outline_logged = True

        # ── 5. Timing metrics ─────────────────────────────────────────────────
        _log_scalar("metrics/timing/t_update_ms", d.get("t_update", 0.0))
        _log_scalar("metrics/timing/t_adam_ms", d.get("t_adam", 0.0))
        _log_scalar("metrics/timing/t_cov_ms", d.get("t_cov", 0.0))
        _log_scalar("metrics/timing/t_breakdown_ms", d.get("t_bkd", 0.0))

        # ── 6. Loss / FE terms ────────────────────────────────────────────────
        if not early:
            _log_scalar("metrics/loss/loss_init", d.get("loss_init", 0.0))
            _log_scalar("metrics/loss/final_loss_adam", d.get("final_loss", 0.0))
            _log_scalar("metrics/loss/final_loss_pred", float("nan"))
            _log_scalar("metrics/loss/l_boundary", d.get("l_bnd", 0.0))
            _log_scalar("metrics/loss/l_obs", d.get("l_obs", 0.0))
            _log_scalar("metrics/loss/l_motion", d.get("l_mot", 0.0))
            _log_scalar("metrics/loss/l_corner", d.get("l_cor", 0.0))
        else:
            _log_scalar("metrics/loss/final_loss_adam", float("nan"))
            _log_scalar("metrics/loss/final_loss_pred", d.get("final_loss", 0.0))

        # ── 7. Quality metrics ────────────────────────────────────────────────
        _log_scalar("metrics/quality/sdf_mse", d.get("sdf_mse", 0.0))
        _log_scalar("metrics/quality/innov_norm", d.get("innov_norm", 0.0))
        _log_scalar("metrics/quality/cov_xx", d.get("cov_xx", 0.0))
        _log_scalar("metrics/quality/cov_yy", d.get("cov_yy", d.get("cov_xx", 0.0)))
        _log_scalar("metrics/quality/cov_tt", d.get("cov_tt", 0.0))
        _log_scalar("metrics/quality/cond_num", d.get("cond_num", 0.0))

        # ── 8. Pipeline state ─────────────────────────────────────────────────
        _log_scalar("metrics/pipeline/window_size", d.get("ws", 0))
        _log_scalar("metrics/pipeline/iters", d.get("iters", 0))
        _log_scalar("metrics/pipeline/early_exit", 1.0 if early else 0.0)

        # ── 9. Pose debug table (text) ───────────────────────────────────────
        pred_x = d.get("pred_x", float("nan"))
        pred_y = d.get("pred_y", float("nan"))
        pred_th = d.get("pred_theta", float("nan"))
        innov_x = d.get("innov_x", float("nan"))
        innov_y = d.get("innov_y", float("nan"))
        innov_th = d.get("innov_theta", float("nan"))
        pose_table = (
            f"mode={'PRED' if early else 'ADAM'} | iters={int(d.get('iters', 0))} | ws={int(d.get('ws', 0))}\n"
            f"pred: x={pred_x:.3f} y={pred_y:.3f} th={pred_th:.3f}\n"
            f"opt : x={x:.3f} y={y:.3f} th={theta:.3f}\n"
            f"innov: dx={innov_x:.3f} dy={innov_y:.3f} dth={innov_th:.3f} norm={d.get('innov_norm', float('nan')):.3f}\n"
            f"uncert: cov_xx={d.get('cov_xx', float('nan')):.4f} cov_tt={d.get('cov_tt', float('nan')):.4f} cond={d.get('cond_num', float('nan')):.2f}"
        )
        if hasattr(rr, "TextDocument"):
            rr.log("metrics/debug/pose_table_doc", rr.TextDocument(pose_table))


# ──────────────────────────────────────────────────────────────────────────────
def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def serve(bridge: RerunBridge, host: str, port: int):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"[bridge] Listening on {host}:{port}", flush=True)

    while True:
        conn, addr = srv.accept()
        print(f"[bridge] Connected: {addr}", flush=True)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            while True:
                hdr = recv_exact(conn, 4)
                if hdr is None:
                    break
                size = struct.unpack_from("<I", hdr)[0]
                if size == 0 or size > 16 * 1024 * 1024:  # sanity: max 16 MB
                    break
                payload = recv_exact(conn, size)
                if payload is None:
                    break
                try:
                    d = json.loads(payload.decode("utf-8"))
                    bridge.process(d)
                except Exception as frame_err:
                    # Keep the socket alive: skip bad frame and continue.
                    print(f"[bridge] Frame dropped due to processing error: {frame_err}", flush=True)
                    traceback.print_exc()
                    continue
        except Exception as e:
            print(f"[bridge] Error: {e}", flush=True)
        finally:
            conn.close()
            print("[bridge] Disconnected, waiting for next connection...", flush=True)


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="sdf_localizer → Rerun bridge")
    parser.add_argument("--port",   type=int, default=PORT,  help="TCP port to listen on")
    parser.add_argument("--host",   type=str, default=HOST,  help="Listen address")
    parser.add_argument("--no-spawn", action="store_true",   help="Don't spawn Rerun viewer (use rr.connect_grpc instead)")
    parser.add_argument("--connect", type=str, default=None, help="gRPC address to connect to (e.g. rerun+http://localhost:9876/proxy)")
    args = parser.parse_args()

    bridge = RerunBridge()

    if args.connect:
        rr.init("sdf_localizer")
        rr.connect_grpc(args.connect)
        rr.send_blueprint(build_blueprint())
        print(f"[bridge] Connected to Rerun at {args.connect}", flush=True)
    else:
        bridge.init_rerun(spawn_viewer=not args.no_spawn)

    serve(bridge, args.host, args.port)
