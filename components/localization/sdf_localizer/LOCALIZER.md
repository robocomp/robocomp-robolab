# SDF Localizer — Mathematical Reference

## 1. Overview

The SDF Localizer estimates the 2-D pose $\mathbf{s}_t = (x, y, \theta)$ of a
robot inside a known room by minimising **Realized Free Energy (RFE)** over a
sliding window of past observations.  The core idea comes from the
*Total-Time Active Inference* formulation (Bustos et al, 2026): keep a fixed-length
window of states $\mathbf{S}_{t-W:t}$ and jointly optimise them so that
the negative log-posterior (free energy) is minimised at every step.

### Threads

| Thread | Rate | Role |
|--------|------|------|
| Lidar Reader | ≈ 17 fps | Reads lidar scans, stores `(points, timestamp)` in a double-buffer |
| Localization (`RoomConcept::run()`) | 5–14 fps | Prediction → window slide → Adam optimisation → covariance update |
| Qt `compute()` | ≈ 10 fps | Forward-projects the latest pose estimate, draws lidar overlay |

---

## 2. Generative Model

The factor graph over a window of $W$ states:

$$
p(\mathbf{S}_{0:W}, \mathbf{O}_{0:W})
= \underbrace{p(\mathbf{s}_0 \mid \mu_\partial, \Sigma_\partial)}_{\text{boundary prior}}
  \;\prod_{k=1}^{W}
    \underbrace{p(\mathbf{s}_k \mid \mathbf{s}_{k-1}, \mathbf{u}_k)}_{\text{motion factor}}
  \;\prod_{k=0}^{W}
    \underbrace{p(\mathbf{o}_k \mid \mathbf{s}_k)}_{\text{observation factor}}
$$

### 2.1 Observation Factor

Each scan $\mathbf{o}_k$ consists of $N$ lidar points $\{\mathbf{p}^{(i)}\}$
measured in the robot frame.  Given the room geometry as a signed distance
field $\mathrm{SDF}(\cdot)$, the observation likelihood is:

$$
-\log p(\mathbf{o}_k \mid \mathbf{s}_k)
\;=\; \frac{1}{2\sigma_{\mathrm{obs}}^2}
     \;\frac{1}{N}\sum_{i=1}^{N}
     \ell_\delta\!\bigl(\mathrm{SDF}(T_k\,\mathbf{p}^{(i)})\bigr)
     \;+\;\mathrm{const}
$$

where:

- $T_k = R(\theta_k)\,\mathbf{p} + \mathbf{t}_k$ transforms robot-frame
  points to the room frame (see §5).
- $\ell_\delta$ is the **Huber loss** with threshold $\delta$:
  $$
  \ell_\delta(z) = \begin{cases}
    \tfrac{1}{2}z^2 & |z| \le \delta \\
    \delta\,(|z| - \tfrac{1}{2}\delta) & |z| > \delta
  \end{cases}
  $$
  This makes the likelihood robust to outlier points (dynamic obstacles,
  sensor noise).
- Default parameters: $\sigma_{\mathrm{obs}} = 0.05\,\text{m}$, $\delta = 0.15\,\text{m}$.

#### Room SDF

Two modes:

- **Polygon SDF** (`sdf_polygon`): For each query point, compute the distance
  to the nearest segment of the room boundary polygon (vectorised over all
  segments).

- **Box SDF** (`sdf_box`): Axis-aligned 3-D box SDF
  $d = \|\max(\mathbf{0},\, |\mathbf{p}| - \mathbf{h})\| + \min(0, \max_i(|\mathbf{p}_i| - h_i))$
  where $\mathbf{h}$ are the half-extents.

### 2.2 Motion Factor

Consecutive states are linked by an odometry-derived delta
$\hat{\boldsymbol\delta}_k \in \mathbb{R}^3$:

$$
-\log p(\mathbf{s}_k \mid \mathbf{s}_{k-1}, \mathbf{u}_k)
\;=\;
\frac{1}{2}
\mathbf{r}_k^\top \,\Sigma_{\mathrm{dyn},k}^{-1}\, \mathbf{r}_k
$$

where the residual is:

$$
\mathbf{r}_k
= \begin{pmatrix}
    x_k - x_{k-1} - \hat\delta_x \\
    y_k - y_{k-1} - \hat\delta_y \\
    \mathrm{wrap}\!\bigl((θ_k - θ_{k-1}) - \hat\delta_\theta\bigr)
  \end{pmatrix}
$$

The $\mathrm{wrap}(\alpha) = \mathrm{atan2}(\sin\alpha,\cos\alpha)$ ensures
the angular residual stays in $[-\pi,\pi]$ and is differentiable for
PyTorch autograd.

#### Odometry Delta Computation

The predicted delta $\hat{\boldsymbol\delta}_k$ is obtained from
**dual-prior fusion** of two independent motion estimates:

1. **Command prior** — integrate commanded velocities $(v_x, v_y, \omega)$
   over the inter-scan interval $[t_{k-1}, t_k]$:

   $$
   \hat{\boldsymbol\delta}^{\mathrm{cmd}} = \int_{t_{k-1}}^{t_k}
   R(\theta_{\mathrm{run}}(t))\,
   \begin{pmatrix} v_x(t) \\ v_y(t) \end{pmatrix} dt
   ,\qquad
   \Delta\theta = \int -\omega\,dt
   $$

2. **Measured odometry prior** — same integration using encoder/IMU readings
   $(\dot{x}_{\mathrm{side}}, \dot{x}_{\mathrm{adv}}, \omega_{\mathrm{meas}})$.

Both come with diagonal covariances
$\Sigma_{\mathrm{cmd}}, \Sigma_{\mathrm{odom}}$ computed from a
noise model:

$$
\sigma_{\mathrm{pos}} = \sigma_{\mathrm{base}} + k_{\mathrm{trans}} \|\Delta \mathbf{t}\|
\,,\quad
\sigma_{\theta} = 0.01 + k_{\mathrm{rot}} |\Delta\theta|
$$

Fusion is Bayesian product-of-Gaussians:

$$
\Sigma_{\mathrm{fused}}^{-1} = \Sigma_{\mathrm{cmd}}^{-1} + \Sigma_{\mathrm{odom}}^{-1}
\,,\qquad
\mu_{\mathrm{fused}} = \Sigma_{\mathrm{fused}}
\bigl(\Sigma_{\mathrm{cmd}}^{-1}\mu_{\mathrm{cmd}}
    + \Sigma_{\mathrm{odom}}^{-1}\mu_{\mathrm{odom}}\bigr)
$$

The fused delta and covariance are stored in each `WindowSlot` as
`odometry_delta` and `motion_cov`.

### 2.3 Boundary Prior

When the window slides forward, the oldest state $\mathbf{s}_0$ is
summarised into a Gaussian prior $\mathcal{N}(\mu_\partial, \Sigma_\partial)$
that anchors the new oldest slot:

$$
-\log p(\mathbf{s}_0 \mid \mu_\partial, \Sigma_\partial)
\;=\;
\frac{1}{2}
\widetilde{\mathbf{r}}_\partial^\top \,\Lambda_\partial\, \widetilde{\mathbf{r}}_\partial
$$

where $\widetilde{\mathbf{r}}_\partial$ has wrapping on the theta component,
$\mu_\partial$ is the MAP estimate of the dropped state, and
$\Lambda_\partial = \Sigma_\partial^{-1}$ is the **precision**.

#### Precision Computation

The precision is the **exact 3×3 Hessian** of a mini-loss at the dropped
state, computed via PyTorch double-backprop (`autograd_hessian_3x3`):

$$
H_{ij} = \frac{\partial^2 \mathcal{L}_{\mathrm{mini}}}{\partial s_i \,\partial s_j}
$$

where the mini-loss includes the SDF observation at the oldest slot,
the motion factor to its successor (if any), and the existing boundary
prior.  The algorithm:

1. **First pass:** `grad = autograd::grad(loss, pose, create_graph=true)` —
   retains the computation graph through the gradient.
2. **Second pass:** for each $i \in \{0,1,2\}$,
   `autograd::grad(grad[i], pose)` yields row $i$ of the Hessian.
3. **Symmetrise:** $H \leftarrow \tfrac{1}{2}(H + H^\top)$.
4. **Eigenvalue clamping:** all eigenvalues are clamped to
   $\geq 10^{-3}$ to guarantee positive-definiteness.

This captures the full $x$-$\theta$ cross-coupling introduced by the
pose-dependent SDF transformation, giving tighter and more accurate
covariance estimates — especially during rotations.

### 2.4 Corner Observation Factor

Room corners provide strong geometric constraints because they are
point landmarks at known world-frame positions.  The corner detector
(`CornerDetector`) runs on the **full** lidar cloud (≈ 4500 pts) before
the early-exit check, so corner observations are available even when
the SDF optimisation is skipped.

#### Detection pipeline

1. **Model-corner filtering.**  From the $N$-vertex room polygon, vertices
   whose interior angle falls outside $[25°, 155°]$ are rejected as flat
   bends.  Each surviving vertex stores its two adjacent wall directions
   $\hat{\mathbf{d}}_{\mathrm{in}}, \hat{\mathbf{d}}_{\mathrm{out}}$.

2. **Projection.**  Each model corner $\mathbf{c}_j$ is projected into the
   robot frame: $\hat{\mathbf{c}}_j = R(-\theta)(\mathbf{c}_j - \mathbf{t})$.
   Corners beyond `max_range` (15 m) are skipped.

3. **Neighbourhood partitioning.**  Lidar points within `search_radius`
   (1.5 m) of the predicted corner are split into two groups by
   perpendicular distance to each wall line through the prediction:
   $$
   \text{group} = \begin{cases}
     \mathrm{in}  & \text{if } |\mathbf{n}_{\mathrm{in}}^\top \mathbf{d}|
                     < |\mathbf{n}_{\mathrm{out}}^\top \mathbf{d}| \\
     \mathrm{out} & \text{otherwise}
   \end{cases}
   $$
   where $\mathbf{d} = \mathbf{p} - \hat{\mathbf{c}}_j$ and
   $\mathbf{n}$ are wall normals.

4. **PCA line fit + intersect.**  A least-squares line (via 2×2 scatter
   matrix eigendecomposition) is fitted to each group.  The two lines are
   intersected; the result is the **detected** corner position
   $\mathbf{z}_{k,j}$ in robot frame.

5. **Quality gates.**  The detection is accepted only if:
   - Each group has $\geq 3$ points.
   - The inter-line angle is in $[25°, 155°]$.
   - $\|\mathbf{z}_{k,j} - \hat{\mathbf{c}}_j\| < 1.5$ m.

6. **Covariance.**  Geometric uncertainty from the line-fit normals:
   $\Sigma_c = \sigma^2 (\mathbf{n}_{\mathrm{in}} \mathbf{n}_{\mathrm{in}}^\top
   + \mathbf{n}_{\mathrm{out}} \mathbf{n}_{\mathrm{out}}^\top)$,
   with $\sigma = 0.06$ m (the RANSAC inlier band width).

#### Corner factor in the RFE

Each accepted corner match produces a factor linking the slot's pose
$\mathbf{s}_k$ to the known model corner $\mathbf{c}_j$:

$$
\hat{\mathbf{z}} = R(-\theta_k)(\mathbf{c}_j - \mathbf{t}_k)
$$

$$
\mathbf{r} = \mathbf{z}_{k,j} - \hat{\mathbf{z}}
$$

$$
\ell_{\mathrm{crn}} =
\frac{1}{2\sigma_{\mathrm{crn}}^2}\;
\ell^{\mathrm{Huber}}_{\delta_c}(\|\mathbf{r}\|)\;
\|\mathbf{r}\|^2
$$

where:

- $\sigma_{\mathrm{crn}} = 0.04$ m (`corner_obs_sigma`).
- $\delta_c = 0.3$ m (`corner_huber_delta`) — the Huber threshold
  saturates large residuals to prevent gross detections from dominating.
- Only the **newest** $N_c = 5$ slots carry corner factors
  (`corner_max_slots`), because older slots' detections were made at
  stale poses and may have drifted.

The Huber weighting is applied as a multiplicative scalar:

$$
\ell^{\mathrm{Huber}}_\delta(r) = \begin{cases}
  1 & r \le \delta \\
  \delta / r & r > \delta
\end{cases}
$$

so that $\ell_{\mathrm{crn}} \propto r^2$ inside the threshold and
$\propto r$ outside it.

### 2.5 Rotation–Position Coupling in Motion Covariance

During pure rotation the translation magnitude is near-zero, which
would make the motion covariance position entries collapse to the
stationary floor ($10^{-3}$ m).  This is unrealistically tight: the
robot still wobbles while pivoting (wheel slip, lever-arm effects).

A **rotation-position coupling** term inflates the position uncertainty
proportionally to the rotation magnitude:

$$
\sigma_{\mathrm{pos}} = \sqrt{\sigma_{\mathrm{base}}^2 + (k_{\mathrm{rp}} |\Delta\theta|)^2}
$$

with $k_{\mathrm{rp}} = 0.15$ m/rad (`rotation_position_coupling`).
At a typical rotation speed of $0.2$ rad per frame this yields
$\sigma_{\mathrm{pos}} \approx 0.03$ m, keeping the motion precision
at $\sim 10^3$ instead of $\sim 10^6$.

---

## 3. Optimisation: Adam over the RFE

The full RFE loss to be minimised is:

$$
\mathcal{F}_{\mathrm{RFE}}(\mathbf{S})
= \underbrace{\ell_\partial(\mathbf{s}_0)}_{\text{boundary}}
+ \sum_{k=0}^{W} \underbrace{\ell_{\mathrm{obs}}(\mathbf{s}_k)}_{\text{SDF obs.}}
+ \sum_{k=1}^{W} \underbrace{\ell_{\mathrm{dyn}}(\mathbf{s}_k, \mathbf{s}_{k-1})}_{\text{motion}}
+ \sum_{k \in \mathcal{W}_c} \sum_{j \in \mathcal{V}_k}
  \underbrace{\ell_{\mathrm{crn}}(\mathbf{s}_k, \mathbf{c}_j, \mathbf{z}_{k,j})}_{\text{corner}}
$$

where $\mathcal{W}_c$ is the set of the newest $N_c$ window slots
(default $N_c = 5$) and $\mathcal{V}_k$ the corners detected in slot $k$.

All $W$ pose tensors $\{\mathbf{s}_k\}$ are optimised jointly using a single
**Adam** optimizer with learning rate $\eta = 0.05$ for up to 25 iterations (with
convergence checks after iteration 5).

### 3.1 Velocity-Adaptive Gradient Scaling

Before each Adam step, the gradient of the **newest** pose is rescaled
by a motion-profile-dependent weight vector
$\mathbf{w} = (w_x, w_y, w_\theta)$:

| Profile | $w_x$ | $w_y$ | $w_\theta$ |
|---------|--------|--------|------------|
| Pure rotation | 0.5 | 0.5 | 2.0 |
| Pure translation (Y-dominant) | 1.0 | 2.0 | 0.5 |
| Pure translation (X-dominant) | 2.0 | 1.0 | 0.5 |
| Combined motion | 1.2 | 1.2 | 1.2 |
| Stationary | 1.0 | 1.0 | 1.0 |

The weights are EMA-smoothed ($\alpha = 0.3$) to avoid abrupt changes.
This acts as a preconditioner that directs Adam's attention to the
degrees of freedom that are actually changing.

### 3.2 Prediction-Based Early Exit

When the predicted pose already fits the current scan well
($\mathrm{SDF}_{\mathrm{mean}} < \sigma_{\mathrm{sdf}} \cdot k_{\mathrm{trust}}$
and the trajectory covariance trace is small and the robot is not
rotating fast), the Adam loop is skipped entirely. This saves
significant compute during smooth straight-line motion.

---

## 4. Posterior Covariance Update

After Adam, the covariance of the newest state is updated using an
information-filter update:

$$
\Lambda_{\mathrm{post}} = \Lambda_{\mathrm{prior}} + H_{\mathrm{lik}} + \lambda I
$$

where:

- $\Lambda_{\mathrm{prior}} = \Sigma_{\mathrm{pred}}^{-1}$ is the prior
  precision from the EKF prediction.
- $H_{\mathrm{lik}}$ is the **exact 3×3 Hessian** of the scaled SDF
  likelihood at the optimised pose, computed via double-backprop
  (`autograd_hessian_3x3`).  Eigenvalues are clamped to $\geq 10^{-4}$
  to guarantee positive-definiteness.
- $\lambda = 10^{-4}$ is Tikhonov regularisation.
- $\Sigma_{\mathrm{post}} = \Lambda_{\mathrm{post}}^{-1}$.

Sanity checks (finite, positive-definite, condition number $< 10^6$)
guard against degenerate covariances.

### EKF Prediction Step

The prediction propagates covariance through the motion model Jacobian:

$$
\Sigma_{\mathrm{pred}} = F\,\Sigma_{\mathrm{prev}}\,F^\top + Q
$$

where $F$ is the state-transition Jacobian:

$$
F = I + \begin{pmatrix}
0 & 0 & -\Delta y_{\mathrm{local}}\sin\theta - \Delta x_{\mathrm{local}}\cos\theta \\
0 & 0 & \;\;\Delta y_{\mathrm{local}}\cos\theta - \Delta x_{\mathrm{local}}\sin\theta \\
0 & 0 & 0
\end{pmatrix}
$$

and $Q$ is the anisotropic process noise transformed to the global frame:

$$
Q = R(\theta) \,
\mathrm{diag}(\sigma_{\mathrm{lat}}^2, \sigma_{\mathrm{fwd}}^2)
\, R(\theta)^\top
\oplus
\sigma_\theta^2
$$

with larger noise in the forward direction than lateral
(differential-drive assumption).

---

## 5. Geometry Conventions

### 5.1 Robot Frame

| Axis | Direction | Velocity Variable |
|------|-----------|-------------------|
| X | **Lateral** (right) | `adv_x` / `side` |
| Y | **Forward** | `adv_z` / `adv` |
| θ | **Heading** in world frame | — |

### 5.2 World Frame

- Origin at the room centre $(0, 0)$.
- Positive X to the right, positive Y upward.
- Positive $\theta$ = counter-clockwise (standard maths convention).

### 5.3 Angular Velocity Sign Convention

Both `integrate_velocity_over_window()` and `integrate_odometry_over_window()`
apply:

$$
\Delta\theta = -\omega \cdot \Delta t
$$

This means a **positive** angular velocity command $\omega > 0$ produces a
**negative** heading change.  This inversion is consistent throughout the
entire pipeline (commanded, measured, fused priors, motion factors).

> **Rationale:** The robot middleware reports $\omega$ with the convention
> that positive means "turn left" in the robot frame, while the world-frame
> heading decreases for a left turn when X=right, Y=forward.

### 5.4 Point Transformation (Robot → Room)

In `sdf_at_pose`:

$$
\mathbf{p}_{\mathrm{room}} = R(\theta)\,\mathbf{p}_{\mathrm{robot}} + \mathbf{t}
$$

where $R(\theta) = \begin{pmatrix}\cos\theta & -\sin\theta \\ \sin\theta & \cos\theta\end{pmatrix}$
is the standard rotation matrix.

### 5.5 Forward Display Projection

The Qt `compute()` loop compensates for the age of the displayed
pose by dead-reckoning with the latest velocity reading.  Both
timestamps are in the **driver/acquisition clock domain**:

$$
\Delta t = \frac{t_{\mathrm{lidar}} - t_{\mathrm{pose}}}{1000}
$$

$$
\begin{pmatrix} \Delta x \\ \Delta y \end{pmatrix}
= R(\theta) \begin{pmatrix} v_{\mathrm{side}} \\ v_{\mathrm{adv}} \end{pmatrix} \Delta t
\,,\qquad
\Delta\theta = \omega \cdot \Delta t
$$

Note: the display projection uses $+\omega$ (no sign flip), because
it compensates a delay that occurred in positive time with the
unmodified velocity direction.

---

## 6. Sliding Window Lifecycle

```
┌──────────────────────────────────────────────────────────────────┐
│                    update() call                                 │
│                                                                  │
│  1. compute_odometry_prior()  ← integrate cmd velocities         │
│  2. compute_measured_odometry_prior()  ← integrate encoder/IMU   │
│  3. predict_step()  ← EKF covariance propagation                 │
│  4. fuse_priors()  ← Bayesian fusion of dual priors              │
│                                                                  │
│  5. (If window full) "cheap slide":                              │
│        boundary_prior_.mu ← MAP of oldest                        │
│        pop_front()                                               │
│                                                                  │
│  6. push_back(new WindowSlot)                                    │
│     - pose = fused prediction (requires_grad=true)               │
│     - lidar_points = current scan                                │
│     - odometry_delta = fused Δ                                   │
│     - motion_cov = fused Σ_dyn                                   │
│                                                                  │
│  7. (Optional early exit if prediction accurate enough)          │
│                                                                  │
│  8. Adam loop (≤25 iters) minimising F_RFE over all W slots     │
│     - velocity-adaptive gradient scaling on newest pose          │
│     - convergence check after iteration 5                        │
│                                                                  │
│  9. Covariance update (exact Hessian via double-backprop)        │
│                                                                  │
│ 10. slide_window_boundary_prior_only()                           │
│     - Recompute Λ_∂ at the (new) oldest slot after Adam          │
│       refined all poses                                          │
│                                                                  │
│ 11. Publish UpdateResult (pose, covariance, sdf_mse, timestamp)  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 7. Recovery & Bootstrap

### Bootstrap

On startup, the localizer:

1. Tries to load a saved seed pose from disk (`last_robot_pose.txt`).
2. If no saved pose, waits for a lidar scan and executes a **grid search**
   over $(x, y, \theta)$ within the room bounds, picking the pose with
   lowest mean absolute SDF.

### Recovery

If the Adam-optimised loss exceeds `recovery_loss_threshold` for
`recovery_consecutive_count` consecutive frames, a full grid search
is triggered automatically to re-localise.

### Manual Pose Reset

- **Shift+Left-click** on the viewer: repositions the robot at
  the clicked world coordinates (keeping current $\theta$).
- **Ctrl+Left-click**: sets $\theta = \mathrm{atan2}(\Delta y, \Delta x)$
  from robot to cursor.

Both use `push_command(CmdSetPose{...})` which is drained on the
localization thread. A manual reset sets `manual_reset_frames_ = 5`,
during which Adam runs without early-exit to let the filter reconverge.

---

## 8. Corner Detection & Factor Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `corner_obs_sigma` | 0.04 m | Isotropic corner measurement noise $\sigma_{\mathrm{crn}}$ |
| `corner_huber_delta` | 0.3 m | Huber threshold for corner residuals |
| `corner_max_slots` | 5 | Number of newest window slots carrying corner factors |
| `min_tracking_steps_for_corners` | 5 | Adam tracking steps required before corners are added |
| `search_radius` | 1.5 m | Neighbourhood radius per model corner |
| `min_points_per_line` | 3 | Min lidar points per wall group |
| `max_match_distance` | 1.5 m | Max detected-vs-predicted corner distance |
| `min_corner_angle` / `max_corner_angle` | 25° / 155° | Inter-wall angle acceptance band |
| `rotation_position_coupling` | 0.15 m/rad | Position uncertainty induced per radian of rotation |

---

## 9. Known Approximations & Limitations

| Approximation | Impact | Notes |
|---------------|--------|-------|
| Velocity-adaptive gradient scaling | Acts as a preconditioner; does not affect Hessian (computed separately post-Adam) | Weights are only applied to the newest pose during Adam steps |
| Motion covariance stored once per slot | Becomes stale if the odometry model changes | Window size $W=10$ ≈ 0.7 s at 14 fps; staleness is bounded |
| Missing log-partition in observation | Constant term has no effect on gradients | Only matters if the loss value is used for information-theoretic diagnostics |
| Eigenvalue clamping on Hessian | Prevents near-zero eigenvalues from inflating covariance | Floor is $10^{-4}$ (posterior) / $10^{-3}$ (boundary prior) |

---

## 10. Parameter Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rfe_window_size` | 10 | Number of past states retained |
| `rfe_obs_sigma` | 0.05 m | SDF observation noise std |
| `rfe_huber_delta` | 0.15 m | Huber loss threshold |
| `learning_rate_pos` | 0.05 | Adam learning rate |
| `num_iterations` | 25 | Max Adam iterations per update |
| `cmd_noise_trans` | 0.20 | Fractional position noise per meter (command) |
| `cmd_noise_rot` | 0.10 | Fractional rotation noise per radian (command) |
| `odom_noise_trans` | 0.08 | Fractional position noise per meter (odometry) |
| `odom_noise_rot` | 0.04 | Fractional rotation noise per radian (odometry) |
| `sigma_sdf` | 0.15 m | SDF noise for early-exit threshold |
| `prediction_trust_factor` | 0.5 | Early-exit multiplier |
| `recovery_loss_threshold` | 0.3 | Loss above which frame is "bad" |
| `recovery_consecutive_count` | 3 | Bad frames before grid-search recovery |

---

## 11. Active Target Selection (Level 1 — Epistemic Planner)

Once the localizer has an estimate $(\mathbf{s}_t, \Sigma_t)$ of the
robot's state, the **epistemic planner** decides *where the robot should
go* to reduce its uncertainty the most.  This is Level 1 of a two-stage
active-inference pipeline implemented in `EpistemicPlanner`.

### 11.1 Candidate Generation

A uniform grid of points at spacing `grid_resolution` (0.25 m) is
generated inside the room polygon.  Candidates are filtered by:

- **Distance band:** $d \in [\texttt{min\_distance},\, \texttt{max\_distance}]$ from the robot (1.0–5.0 m).
- **Inside polygon:** `point_in_polygon()` rejects positions outside
  non-convex room boundaries.
- **Wall margin:** candidates closer than `target_wall_margin` (1.0 m)
  to any polygon edge are discarded to keep the robot away from walls.
- **Count cap:** at most `max_candidates` (200) survive.

### 11.2 Angular Dominance Check

Before evaluating candidates, the planner checks whether heading
uncertainty dominates position uncertainty:

$$
\frac{\sigma_\theta^2}{\max(\sigma_x^2,\, \sigma_y^2)} > \tau_{\mathrm{ang}}
$$

with $\tau_{\mathrm{ang}} = 50$.  If so, the best action is to
**rotate in place** rather than translate; a single synthetic target
at the robot's current position with `rotate_in_place = true` is
returned immediately.

### 11.3 FIM-Based D-Optimality Scoring

For each candidate position $\mathbf{p}$, the planner:

1. **Corner visibility test.**  Determines which room polygon corners are
   visible from $\mathbf{p}$ using a segment-intersection occlusion test
   (`is_corner_visible`), with a maximum range of 10 m.

2. **Assumed arrival heading.**  The heading at $\mathbf{p}$ is predicted
   as $\hat\theta = \mathrm{atan2}(-\Delta x, \Delta y)$ (the direction
   of travel in our $\theta = 0 \to Y^+$ convention).

3. **Corner observation Jacobian.**  For each visible corner
   $\mathbf{c}_j$ the $2 \times 3$ observation Jacobian is:

   $$
   J_j = \frac{\partial \mathbf{z}_j}{\partial (x, y, \theta)}
   = \begin{pmatrix}
     -\cos\theta & -\sin\theta & z_1 \\
      \sin\theta & -\cos\theta & -z_0
   \end{pmatrix}
   $$

   where $z_0 = \cos\theta\,\Delta x + \sin\theta\,\Delta y$,
   $z_1 = -\sin\theta\,\Delta x + \cos\theta\,\Delta y$,
   $\Delta\mathbf{x} = \mathbf{c}_j - \mathbf{p}$.

4. **Fisher Information Matrix.**  The summed FIM from all $M$ visible
   corners is:

   $$
   I_{\mathrm{crn}} = \frac{1}{\sigma_{\mathrm{crn}}^2}
   \sum_{j=1}^{M} J_j^\top J_j
   $$

   with $\sigma_{\mathrm{crn}} = 0.04$ m.

5. **D-optimality gain.**  The information-gain score is:

   $$
   \Delta_D(\mathbf{p})
   = \log\det\!\bigl(\Lambda_{\mathrm{prior}} + I_{\mathrm{crn}}\bigr)
   - \log\det\!\bigl(\Lambda_{\mathrm{prior}}\bigr)
   $$

   where
   $\Lambda_{\mathrm{prior}} = (\Sigma_t + 10^{-6}I)^{-1}$
   is the current prior precision.  Higher $\Delta_D$ means that
   moving to $\mathbf{p}$ would yield the largest posterior precision
   increase.

Candidates are sorted by composite score (descending), and the top candidate
becomes the planner's **current target**.  The target is held fixed until
the robot arrives within `arrival_distance` (0.15 m), at which point
it dwells for `dwell_time` (2 s) and then selects a new target.

### 11.4 Exploration Distance Bonus

Pure FIM scoring is nearly distance-invariant: a candidate 1 m away sees
the same corners at similar angles as one 4 m away, producing equal
$\Delta_D$.  To break this tie in favour of exploration, the raw FIM gain
is scaled by a saturating distance factor:

$$
f_{\mathrm{dist}}(d) = 1 + \tanh(w_{\mathrm{exp}} \cdot d)
$$

with $w_{\mathrm{exp}} = 0.3$.  At $d = 1$ m the multiplier is 1.29;
at $d = 4$ m it is 1.85.  This ensures that a truly informative nearby
position can still win, but equal-FIM candidates are resolved in favour
of the farther one.

### 11.5 Inhibition of Return (Visit Grid)

To prevent the robot from circling the same region indefinitely, a
**spatial visit grid** implements an inhibition-of-return (IoR) mechanism
inspired by attentional neuroscience.

#### Data structure

A coarse grid of cells (`ior_cell_size` = 0.5 m) covers the room.
Each cell stores the last time $t_{\mathrm{visit}}$ the robot occupied it.
All cells start at epoch (maximally stale).  The grid is initialised once
when `set_room_bounds()` is called, and the robot's current cell is
stamped at the beginning of every `plan()` call.

#### Staleness

The staleness of a candidate position is:

$$
\mathrm{staleness}(\mathbf{p})
= \min\Bigl(1,\;
  \frac{t_{\mathrm{now}} - t_{\mathrm{visit}}(\mathbf{p})}
       {\tau_{\mathrm{decay}}}\Bigr)
$$

- **Never visited** → staleness $= 1$ (maximally attractive).
- **Just visited** → staleness $\approx 0$ (suppressed).
- **After $\tau_{\mathrm{decay}}$ seconds** → staleness recovers to $1$
  (full attraction restored).

Default decay time is $\tau_{\mathrm{decay}} = 30$ s.

#### Score integration

The final composite score for each candidate combines FIM gain,
distance bonus, and IoR staleness:

$$
\mathrm{score}(\mathbf{p})
= \Delta_D(\mathbf{p})
  \;\times\; f_{\mathrm{dist}}(d)
  \;\times\; \bigl(1 + w_{\mathrm{ior}} \cdot \mathrm{staleness}(\mathbf{p})\bigr)
$$

With $w_{\mathrm{ior}} = 2.0$, a fully stale region receives a $3\times$
score boost over a freshly visited one with identical FIM gain, strongly
driving the robot to explore the entire room.

### 11.6 Target Lifecycle

```
current_target_ = ∅
          │
          ▼
  ┌───────────────┐     No target    ┌──────────────────┐
  │  plan() call  │────────────────► │ Level 1: select  │
  └───────┬───────┘                  │  highest ΔD      │
          │ has target               └────────┬─────────┘
          ▼                                   │
  ┌─────────────────┐                         ▼
  │ d < arrival_d?  │──── yes ────►  Dwell (2 s), then clear
  └────────┬────────┘
           │ no
           ▼
    Level 2 (EFE controller)
```

---

## 12. Arc Trajectory Controller (Level 2 — EFE Evaluation)

Given a target from Level 1, Level 2 generates a set of constant-command
**arc trajectories**, rolls them out through the kinematic model,
evaluates the **Expected Free Energy (EFE)** of each, and returns the
first command of the minimum-EFE arc.

### 12.1 Arc Policy Generation

The controller generates $K$ arcs (default $K = 9$, plus 4 pure rotations),
each defined by a constant $(\mathrm{adv}_y, \omega)$ pair held for the
entire horizon of $H = 20$ steps at $\Delta t = 0.2$ s (4 s total).

#### Nominal command

The direction to the target is computed in the body frame
$\mathbf{t}_b = T_{\mathrm{robot}}^{-1}\,\mathbf{p}_{\mathrm{target}}$,
and the nominal speed and heading rate are:

$$
\alpha = \mathrm{atan2}(-t_{b,x},\; t_{b,y})
$$

$$
v_{\mathrm{nom}} = \min\!\Bigl(\frac{d}{T_{\mathrm{hor}}},\; v_{\max}\Bigr)
\cdot \max(0,\, \cos\alpha)
$$

$$
\omega_{\mathrm{nom}} = \mathrm{clamp}\bigl(k_{\mathrm{rot}} \cdot \alpha,\;
-\omega_{\max},\; \omega_{\max}\bigr)
$$

The $\cos\alpha$ alignment factor reduces forward speed when the target
is to the side, preventing the robot from overshooting laterally.

#### Arc spread

The $K$ arcs distribute rotation uniformly around $\omega_{\mathrm{nom}}$:

$$
\omega_i = \mathrm{clamp}\!\Bigl(
  \omega_{\mathrm{nom}} + f_i \cdot \omega_{\max},\;
  -\omega_{\max},\; \omega_{\max}\Bigr)
\,,\quad
f_i = \frac{2i}{K-1} - 1
\,,\quad i = 0 \ldots K\!-\!1
$$

Speed is reduced for arcs deviating from the nominal heading:

$$
v_i = v_{\mathrm{nom}} \cdot \max\!\bigl(0,\; 1 - 0.5\,|f_i|\bigr)
$$

Four additional **pure rotations** $\omega \in \{-1, -0.5, 0.5, 1\} \cdot \omega_{\max}$
cover spin-in-place exploration.

If the target has `rotate_in_place = true` (angular dominance from Level 1),
only rotation policies are generated.

### 12.2 Kinematic Rollout

Each arc policy is integrated through the unicycle kinematic model:

$$
\begin{pmatrix} x_{t+1} \\ y_{t+1} \\ \theta_{t+1} \end{pmatrix}
=
\begin{pmatrix} x_t \\ y_t \\ \theta_t \end{pmatrix}
+
\begin{pmatrix}
  (v_x \cos\theta_t - v_y \sin\theta_t)\,\Delta t \\
  (v_x \sin\theta_t + v_y \cos\theta_t)\,\Delta t \\
  \omega\,\Delta t
\end{pmatrix}
$$

producing a predicted state trajectory $\{\mathbf{s}_0, \ldots, \mathbf{s}_H\}$
of $H+1$ poses.

### 12.3 Expected Free Energy

Each policy $\pi_i$ is scored by its EFE, decomposed into four additive terms:

$$
G(\pi_i) = -w_e \cdot \mathcal{E}(\pi_i)
           + w_p \cdot \mathcal{P}(\pi_i)
           + w_o \cdot \mathcal{O}(\pi_i)
           + \mathcal{B}(\pi_i)
$$

The policy with minimum $G$ is selected.

#### Pragmatic value $\mathcal{P}$

Accumulated squared distance to the target plus a heading-alignment penalty:

$$
\mathcal{P} = \sum_{t=1}^{H}\Bigl[
  \|\mathbf{p}_t - \mathbf{p}_{\mathrm{target}}\|^2
  + w_h \cdot \alpha_t^2
\Bigr]
$$

where $\alpha_t = \mathrm{atan2}(-b_x, b_y)$ is the heading error in the
body frame at step $t$, and $w_h = 2.0$ (`w_heading`).
This drives the robot both toward the target and facing it.

#### Epistemic value $\mathcal{E}$

The FIM-based D-optimality gain (same computation as Level 1 §11.3) evaluated
at the **final** predicted state $\mathbf{s}_H$:

$$
\mathcal{E} = \Delta_D(\mathbf{p}_H,\, \hat\theta_H)
$$

This rewards trajectories that end in positions with high corner visibility
and thus high information gain, directly linking action selection to
uncertainty reduction.

#### Obstacle cost $\mathcal{O}$

A repulsive potential from floor-projected lidar points.  Before scoring,
lidar points that lie within `wall_filter_margin` (0.30 m) of any polygon
edge are removed — these are wall reflections, not real obstacles.

For each surviving obstacle point $\mathbf{l}_j$ and each trajectory step:

$$
\mathcal{O}_t = \min\!\Bigl(
  \sum_{\|\mathbf{p}_t - \mathbf{l}_j\| < r_{\mathrm{obs}}}
  \frac{k_{\mathrm{obs}}}{d^2_{t,j}}
  \;,\;\; C_{\max}
\Bigr)
$$

$$
\mathcal{O} = \sum_{t=1}^{H} \mathcal{O}_t
$$

with $r_{\mathrm{obs}} = 0.35$ m, $k_{\mathrm{obs}} = 0.5$,
$C_{\max} = 10$ per step.  The per-step cap prevents a single cluster
of lidar points from completely dominating the EFE.

#### Boundary penalty $\mathcal{B}$

A flat penalty $w_b = 10$ is added for each step that falls outside the
room polygon (`point_in_polygon` test), strongly discouraging trajectories
that leave the room.

### 12.4 Fallback Controller

If no arc produces a valid policy (empty commands), a simple proportional
controller is used:

$$
\omega = \mathrm{clamp}(k_{\mathrm{rot}} \cdot \alpha_{\mathrm{err}},\;
         -\omega_{\max},\; \omega_{\max})
\,,\qquad
v = v_{\max} \cdot \exp\!\Bigl(-\frac{\alpha_{\mathrm{err}}^2}{2\sigma_g^2}\Bigr)
$$

with $k_{\mathrm{rot}} = 2.0$ and $\sigma_g = 0.5$ rad, giving a
Gaussian speed profile that peaks when the heading error is near zero.

### 12.5 Visualisation

The viewer draws all candidate arc trajectories as faded lilac polylines
and the winning (minimum-EFE) arc in bold magenta.  This provides immediate
visual feedback on which options the controller considered and why the
chosen arc was preferred.

### 12.6 Parameter Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_resolution` | 0.25 m | Candidate target spacing |
| `min_distance` | 1.0 m | Min candidate distance from robot |
| `max_distance` | 5.0 m | Max candidate distance from robot |
| `target_wall_margin` | 1.0 m | Min distance from candidate to polygon edge |
| `angular_dominance_ratio` | 50.0 | $\sigma_\theta^2 / \sigma_{\mathrm{pos}}^2$ threshold for rotate-in-place |
| `w_exploration` | 0.3 | Distance bonus weight (exploration) |
| `ior_cell_size` | 0.5 m | Spatial resolution of the visit grid |
| `ior_decay_time` | 30.0 s | Seconds until visit suppression fully decays |
| `w_ior` | 2.0 | Inhibition-of-return staleness weight |
| `fim_corner_sigma` | 0.04 m | Corner detection noise for FIM |
| `num_arc_curvatures` | 9 | Number of discrete arc curvatures $K$ |
| `horizon_steps` | 20 | Planning horizon steps $H$ |
| `dt` | 0.2 s | Time step per horizon step |
| `max_adv_speed` | 0.3 m/s | Max translational speed $v_{\max}$ |
| `max_rot_speed` | 0.5 rad/s | Max angular speed $\omega_{\max}$ |
| `w_epistemic` | 1.0 | EFE weight for information gain |
| `w_pragmatic` | 1.0 | EFE weight for goal distance |
| `w_heading` | 2.0 | Heading alignment weight inside pragmatic |
| `w_boundary` | 10.0 | Penalty per step outside room bounds |
| `w_obstacle` | 2.0 | EFE weight for obstacle cost |
| `obstacle_radius` | 0.35 m | Interaction radius for lidar obstacles |
| `obstacle_k` | 0.5 | Repulsive potential gain |
| `obstacle_step_cap` | 10.0 | Max obstacle cost per step |
| `wall_filter_margin` | 0.30 m | Lidar wall-point filter distance |
| `arrival_distance` | 0.15 m | Target-reached threshold |
| `dwell_time` | 2.0 s | Pause duration after reaching a target |
| `k_rot` | 2.0 | Proportional gain for heading (fallback) |
| `gaussian_sigma` | 0.5 rad | Speed-profile $\sigma$ (fallback) |
