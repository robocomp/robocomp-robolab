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
   - **Orientation consistency:** each PCA-fitted line direction must align
     with its model edge direction (in robot frame) within
     `max_orientation_dev` (20°).  Formally,
     $|\hat{\mathbf{d}}_{\mathrm{PCA}} \cdot \hat{\mathbf{d}}_{\mathrm{model}}| \geq \cos(20°) \approx 0.940$
     for both the *in*-wall and *out*-wall directions.
     This rejects false matches where the fitted wall angle is correct
     but the wall orientations are flipped or rotated.
   - **Convexity consistency:** the model corner's convexity sign (2D
     cross product $\hat{\mathbf{d}}_{\mathrm{in}} \times
     \hat{\mathbf{d}}_{\mathrm{out}}$, positive for CCW turns) must
     agree with the detected corner's cross product computed from the
     oriented PCA directions.  PCA eigenvectors are oriented using the
     sign of their dot product with the model edge direction, making the
     cross product sign unambiguous.  The gate requires
     $$s_{\mathrm{model}}\;s_{\mathrm{detected}} \;\geq\;
       0.50\,|s_{\mathrm{model}}|$$
     where $s = \hat{\mathbf{d}}_{\mathrm{in}} \times
     \hat{\mathbf{d}}_{\mathrm{out}}$.  This prevents convex corners
     (e.g. room protrusions) from matching concave corners (e.g. room
     recesses) and also rejects ambiguously flat detections whose cross
     product magnitude is too small.

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
| `max_orientation_dev` | 20° | Max angular deviation between PCA line and model edge direction |
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

A uniform grid of points at spacing `grid_resolution` (0.5 m) is
generated inside the room polygon.  Candidates are filtered by:

- **Minimum distance:** $d \geq \texttt{min\_distance}$ (1.0 m) from the
  robot.
- **Inside polygon:** `point_in_polygon()` rejects positions outside
  non-convex room boundaries.  There is no upper distance limit — the
  polygon boundary and wall margin naturally bound the candidates.
- **Wall margin:** candidates closer than `target_wall_margin` (1.0 m)
  to any polygon edge are discarded to keep the robot away from walls.
- **Count cap:** at most `max_candidates` (2000) survive.  With 0.5 m
  resolution the cap is rarely reached, ensuring the **entire** room
  is covered uniformly.

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

Candidates are scored by composite score (descending).  Rather than
greedy top-1 selection, the planner draws a **weighted random sample**
from all candidates using `std::discrete_distribution`, where each
candidate's probability is proportional to its composite score.  This
ensures high-scoring cells are likely to be chosen while still allowing
occasional exploration of lower-ranked alternatives, preventing
repetitive cycling through the same cells.

### 11.4 Exploration Distance Bonus

Pure FIM scoring is nearly distance-invariant: a candidate 1 m away sees
the same corners at similar angles as one 4 m away, producing equal
$\Delta_D$.  To break this tie in favour of exploration, the raw FIM gain
is scaled by a **linear** distance factor:

$$
f_{\mathrm{dist}}(d) = 1 + w_{\mathrm{exp}} \cdot d
$$

with $w_{\mathrm{exp}} = 0.5$.  At $d = 2$ m the multiplier is 2.0;
at $d = 6$ m it is 4.0.  The linear (non-saturating) shape
ensures that distant candidates always retain a meaningful advantage,
preventing the robot from lingering in one portion of the room.

### 11.5 Inhibition of Return (Visit Grid)

To prevent the robot from circling the same region indefinitely, a
**spatial visit grid** implements an inhibition-of-return (IoR) mechanism
inspired by attentional neuroscience.

#### Data structure

A coarse grid of cells (`ior_cell_size` = 0.5 m) covers the room.
Each cell stores two attributes:

1. **Last visit time** $t_{\mathrm{visit}}$ — stamped whenever the robot
   occupies the cell.
2. **FIM information gain** — an exponential moving average of the
   D-optimality gain $\Delta_D$ computed for candidates falling in this
   cell, updated every `evaluate_targets()` call with smoothing
   factor $\alpha = 0.3$:
   $$g_{\mathrm{new}} = (1 - \alpha)\,g_{\mathrm{old}} + \alpha\,\Delta_D$$
   This captures a persistent spatial map of where information is rich.

All cells start at epoch time (maximally stale) and zero FIM gain.
The grid is initialised once when `set_room_bounds()` is called, and
the robot's current cell is stamped at the beginning of every
`plan()` call.

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

Default decay time is $\tau_{\mathrm{decay}} = 120$ s.

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

#### Score grid visualisation

The combined cell scores (FIM gain $\times$ IoR bonus) are exposed via
`cell_scores()` and drawn on the 2D canvas as a soft overlay.  Each cell
is a semi-transparent rectangle with Z-order −5 (behind all other items).
The colour ramp interpolates from **pale blue** (low score) to **warm
orange** (high score), with opacity 40–100.  This gives the operator an
immediate view of where the planner considers information-rich and stale.

### 11.6 Target Lifecycle

```
current_target_ = ∅
          │
          ▼
  ┌───────────────┐     No target    ┌──────────────────┐
  │  plan() call  │────────────────► │ Level 1: weighted │
  └───────┬───────┘                  │  random sample    │
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

### 12.5 Perceptual Bandwidth Speed Limit

Sharp turns cause rapid scene change that can spike the observation
residuals (and thus the free energy) before the optimiser has time to
converge.  To model a **finite perceptual bandwidth**, the controller
couples rotation and translation so that high yaw-rate automatically
reduces the allowed translational speed.

The effective maximum translation speed is:

$$
v_{\max}^{\mathrm{eff}} = v_{\max} \cdot
  \max\!\Bigl(0,\;
    1 - \beta\,\frac{|\omega|}{\omega_{\max}}\Bigr)
$$

where $\beta$ = `bandwidth_coupling` (default 0.7).  At full rotation
($|\omega| = \omega_{\max}$) the robot can still translate at 30 % of
its nominal speed; at zero rotation, full speed is available.

The coupling is enforced at **three points**:

1. **Arc generation** — each arc command is passed through
   `apply_speed_limit()` before rollout, so the predicted trajectories
   reflect the actual kinematics the robot will execute.
2. **Best-policy output** — the winning command is clamped before being
   sent to the base.
3. **Fallback controller** — the proportional controller output is
   clamped identically.

When both `adv_x` and `adv_y` are non-zero the translation vector is
scaled proportionally (preserving direction) so that
$\sqrt{v_x^2 + v_y^2} \le v_{\max}^{\mathrm{eff}}$.

### 12.6 Reactive Speed Governor

While the bandwidth limit (§12.5) couples rotation to translation at the
kinematic level, the **reactive speed governor** provides a second,
independent scaling layer based on **localization quality** measured by
the SDF mean-squared error.

Before each planning cycle the latest `sdf_mse` is fed to
`set_localization_quality()`, which computes a governor factor:

$$
\alpha = \mathrm{clamp}\!\left(
  1 - \frac{\text{sdf\_mse} - \text{sdf\_safe}}
           {\text{sdf\_danger} - \text{sdf\_safe}},\;
  \alpha_{\min},\; 1\right)
$$

where `sdf_safe` and `sdf_danger` define the comfortable and degraded
MSE thresholds, and $\alpha_{\min}$ = `governor_alpha_min` (default
0.2) lower-bounds the scaling to prevent a full stop.

Both $v_{\max}$ and $\omega_{\max}$ are multiplied by $\alpha$ at
the start of `apply_speed_limit()`, so the governor acts *before* the
bandwidth coupling.  The two mechanisms compose multiplicatively:

$$
v_{\max}^{\mathrm{final}} = \alpha \cdot v_{\max} \cdot
  \max\!\bigl(0,\;1 - \beta\,|\omega|/\omega_{\max}\bigr)
$$

The current governor alpha is shown via the velocity label colour
in the UI (red when $\alpha < 0.5$, orange when $\alpha < 0.95$,
default otherwise).

### 12.7 Acceleration Ramp

The planner's output commands can change abruptly between cycles,
especially when a new target is selected or during sharp turns.  To
prevent unbounded angular (and translational) acceleration, a
per-component ramp filter is applied to every command **before** it is
sent to `setSpeedBase()`.

For each velocity component $u \in \{v_x, v_y, \omega\}$:

$$
u_{\mathrm{out}} = u_{\mathrm{prev}} +
  \mathrm{clamp}\!\bigl(u_{\mathrm{desired}} - u_{\mathrm{prev}},\;
  -a_{\max}\,\Delta t,\; +a_{\max}\,\Delta t\bigr)
$$

where $\Delta t$ is the real elapsed time since the last command
(measured via `steady_clock`), and $a_{\max}$ is the per-channel
acceleration limit:

| Channel | Limit | Effect at 17 Hz |
|---------|-------|-----------------|
| Translation ($v_x$, $v_y$) | `max_lin_accel` = 1.5 m/s² | ≈ 0.09 m/s per cycle |
| Rotation ($\omega$) | `max_rot_accel` = 3.0 rad/s² | ≈ 0.18 rad/s per cycle |

The ramp state (`prev_cmd_`) resets to zero when self-targeting is
toggled off, so the robot ramps cleanly from rest when restarted.

### 12.8 Visualisation

The viewer draws all candidate arc trajectories as faded lilac polylines
and the winning (minimum-EFE) arc in bold magenta.  This provides immediate
visual feedback on which options the controller considered and why the
chosen arc was preferred.

### 12.9 Parameter Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_resolution` | 0.5 m | Candidate target spacing |
| `min_distance` | 1.0 m | Min candidate distance from robot |
| `max_candidates` | 2000 | Max evaluated candidates (ensures full room coverage) |
| `target_wall_margin` | 1.0 m | Min distance from candidate to polygon edge |
| `angular_dominance_ratio` | 50.0 | $\sigma_\theta^2 / \sigma_{\mathrm{pos}}^2$ threshold for rotate-in-place |
| `w_exploration` | 0.5 | Linear distance bonus weight (exploration) |
| `ior_cell_size` | 0.5 m | Spatial resolution of the visit grid |
| `ior_decay_time` | 120.0 s | Seconds until visit suppression fully decays |
| `w_ior` | 2.0 | Inhibition-of-return staleness weight |
| `fim_corner_sigma` | 0.04 m | Corner detection noise for FIM |
| `num_arc_curvatures` | 9 | Number of discrete arc curvatures $K$ |
| `horizon_steps` | 20 | Planning horizon steps $H$ |
| `dt` | 0.2 s | Time step per horizon step |
| `max_adv_speed` | 0.6 m/s | Max translational speed $v_{\max}$ |
| `max_rot_speed` | 0.5 rad/s | Max angular speed $\omega_{\max}$ |
| `bandwidth_coupling` | 0.7 | Rotation–translation coupling $\beta$ (§12.5) |
| `sdf_safe` | 0.04 | SDF-MSE below which governor gives full speed (§12.6) |
| `sdf_danger` | 0.025 | SDF-MSE at/above which governor gives $\alpha_{\min}$ (§12.6) |
| `governor_alpha_min` | 0.2 | Minimum speed fraction under governor (§12.6) |
| `max_lin_accel` | 1.5 m/s² | Translational acceleration ramp limit (§12.7) |
| `max_rot_accel` | 3.0 rad/s² | Angular acceleration ramp limit (§12.7) |
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

---

## 13. Time Synchronisation

The localizer runs three concurrent threads that produce and consume
timestamped data at different rates.  A common **epoch-millisecond**
clock (`std::chrono::system_clock`) is the single time reference shared
by all buffers, so every datum — lidar scan, velocity command, odometry
reading, and estimated pose — can be placed on the same timeline.

### 13.1 Data Sources and Timestamps

| Source | Rate | Timestamp | Buffer |
|--------|------|-----------|--------|
| Lidar (HELIOS) | ≈ 17 Hz | `system_clock::now()` at reception + driver-side `data.timestamp` | `lidar_buffer` (triple-slot: GT pose, points, obstacles) |
| Velocity command (joystick / planner) | event-driven | `recv_ts_ms` = `system_clock::now()` at reception; `source_ts_ms` = 0 | `velocity_buffer_` |
| Odometry (FullPoseEstimation) | ≈ 10 Hz | `source_ts_ms` = FPE sensor-side epoch-ms; `recv_ts_ms` = local reception | `odometry_buffer_` |
| Estimated pose (localization output) | 5–14 Hz | `timestamp_ms` = lidar epoch-ms of the scan used | `last_result_` (mutex-guarded) |

All three buffers (`SensorBuffer`, `VelocityBuffer`, `OdometryBuffer`)
are **lock-free** `BufferSync` instances with single-writer / single-reader
semantics.  The only mutex-guarded datum is the `last_result_` shared
between the localization and display threads.

### 13.2 Lidar Reader Thread

```
poll Lidar3D at adaptive period (~25–50 ms)
  ├── timestamp reception with system_clock epoch-ms
  ├── store driver timestamp alongside points
  └── BufferSync.put<0,1,2> atomically (GT pose, lidar points, obstacles)
```

The reader adapts its polling period to the driver's reported cadence
(±2 ms hysteresis around `data.period`), preventing both busy-wait and
missed scans.

### 13.3 Localization Thread — Window Matching

The localization loop (`RoomConcept::run`) is paced entirely by
**new lidar scans**: it reads the latest buffer entry and skips the
iteration if the lidar timestamp has not changed since the last cycle.

When a new scan arrives with timestamp $t_n$, the thread:

1. **Snapshots** the full velocity and odometry histories from their
   respective buffers (lock-free `get_snapshot<0>()`).

2. **Clips** each velocity / odometry command to the time window
   $[t_{n-1},\, t_n]$, where $t_{n-1}$ is the previous scan's timestamp.

3. **Integrates** the clipped commands using a midpoint-angle rule to
   produce a 3-DOF motion prior $\hat{\boldsymbol\delta} = (\Delta x,
   \Delta y, \Delta\theta)$:

   $$
   \theta_{\mathrm{mid}} = \theta_{\mathrm{running}} + \tfrac{1}{2}\,\omega\,\Delta t
   $$

   $$
   \Delta x \mathrel{+}= (v_x \cos\theta_{\mathrm{mid}} - v_y \sin\theta_{\mathrm{mid}})\,\Delta t
   \,,\quad
   \Delta y \mathrel{+}= (v_x \sin\theta_{\mathrm{mid}} + v_y \cos\theta_{\mathrm{mid}})\,\Delta t
   $$

   This integrates **all** commands whose validity interval overlaps the
   scan-to-scan window, weighting each by its overlap duration.

4. **Computes a motion covariance** that scales with the magnitude of the
   integrated motion, including a rotation–position coupling term (wheel
   slip).

When no velocity or odometry data is available in the window, the motion
prior defaults to zero displacement with a tight stationary covariance.

### 13.4 Timestamp Flow Diagram

```
Lidar Reader          Joystick / Planner         FPE Odometry
  ≈17 Hz                 event-driven              ≈10 Hz
    │                        │                        │
    │  epoch-ms              │  epoch-ms              │  epoch-ms
    ▼                        ▼                        ▼
┌────────────┐       ┌──────────────┐        ┌──────────────┐
│lidar_buffer│       │velocity_buf_ │        │odometry_buf_ │
│ (lock-free)│       │ (lock-free)  │        │ (lock-free)  │
└─────┬──────┘       └──────┬───────┘        └──────┬───────┘
      │                     │                       │
      └──────────┬──────────┴───────────────────────┘
                 │
                 ▼
      ┌─────────────────────┐
      │  Localization Thread │  (paced by new lidar scans)
      │  read_last(lidar)   │
      │  snapshot(vel,odom) │
      │  clip to [t_{n-1}, t_n]
      │  integrate → motion prior
      │  predict → Adam → pose
      └──────────┬──────────┘
                 │  last_result_ {pose, timestamp_ms = t_n}
                 │  (mutex)
                 ▼
      ┌─────────────────────┐
      │  Display Thread      │  ≈10 Hz (Qt compute())
      │  get_last_result()  │
      │  forward_project()  │
      └─────────────────────┘
```

### 13.5 Forward Projection (Display Lag Compensation)

The display thread (`compute()`) runs at ≈10 Hz, whereas the localization
result may refer to a lidar scan that is one or two frames old.  To avoid
visible jitter the display thread **forward-projects** the estimated pose
from the result timestamp $t_{\mathrm{loc}}$ to the current lidar
timestamp $t_{\mathrm{lidar}}$:

$$
\mathrm{lag} = t_{\mathrm{lidar}} - t_{\mathrm{loc}}
$$

$$
\hat{\mathbf{s}}_{\mathrm{display}} = \hat{\mathbf{s}}_{\mathrm{loc}}
  + \mathbf{v}_{\mathrm{smooth}} \cdot \mathrm{lag}
$$

The velocity used for extrapolation is obtained from the latest odometry
reading (preferred) or velocity command (fallback), smoothed with an
**exponential moving average** ($\alpha = 0.35$) to suppress sensor jitter:

$$
\bar{v} \leftarrow \alpha\, v_{\mathrm{new}} + (1 - \alpha)\, \bar{v}
$$

### 13.6 Prediction Early Exit

When the robot moves smoothly and the predicted pose (from the motion
prior alone) already produces a low mean SDF error
($\overline{|\mathrm{SDF}|} < \sigma_{\mathrm{obs}} \times 0.5$), the
Adam optimisation is **skipped entirely** for that frame.  This
effectively doubles the throughput of the localization loop during
straight-line motion, freeing CPU for the epistemic planner.

### 13.7 Key Design Choices

- **No lidar point interpolation.**  Motion is accounted for by the
  odometry prior applied *before* optimisation, not by warping scan
  points.
- **Dual timestamps on odometry.**  Both the sensor-side
  (`source_ts_ms`) and local-reception (`recv_ts_ms`) timestamps are
  stored, allowing future transport-delay compensation.
- **Scan-paced loop.**  The localization thread has no fixed-rate timer;
  it naturally runs at the lidar cadence and gracefully degrades when
  lidar drops occur (the `read_last()` call simply returns the most
  recent available scan).
- **Epoch-millisecond everywhere.**  Integer `system_clock` epoch-ms is
  used for all buffer digests and window arithmetic, avoiding
  floating-point drift and making cross-component timestamp comparison
  trivial.

---

## 14. Online Motion Model Learning

### 14.1 Motivation

The motion factor precision matrix $\boldsymbol{\Lambda}_k = \Sigma_{\mathrm{mot},k}^{-1}$
is computed from three static parameters (`EncoderRotSlipK`, `OdomNoiseTrans`,
odometry bias = 0).  In practice these values vary across robots and environments.
The online learner continuously estimates better values without requiring
manual re-tuning.

### 14.2 Architecture Overview

`adapt_motion_model()` is called once per frame after the L-BFGS pass.
It runs **two independent estimators** with different strategies, plus a
bias accumulator:

| Estimator | Strategy | Reason |
|-----------|----------|--------|
| Slip-k $\hat{k}_s$ | Per-pair, newest slot only | Needs instantaneous angular speed; internal pairs have near-zero residuals under `SdfCurrentSlotOnly` |
| Trans-noise $\hat{\sigma}_T$ | Window-span, variance buffer | Single 55 ms slot covers only 0.02–0.05 m; span over W-1 slots gives 4× better SNR |
| Odometry bias $\hat{\mathbf{b}}$ | Mean residual, slow EMA | Captures systematic drift over all good pairs |

All updates share a **separate quality gate**
$\mathrm{SDF\_MSE} < \tau_q^{\mathrm{learn}}$
(default 0.12 m) that is intentionally more permissive than the
boundary Hessian gate ($\tau_q^{\mathrm{Hess}} = 0.08$ m).  Using the
tighter Hessian gate would starve the learner because typical
$\mathrm{SDF\_MSE}$ while moving is 0.06–0.09 m.

### 14.3 Motion Residuals

After each L-BFGS run the window holds optimised poses
$\hat{\mathbf{s}}_0,\ldots,\hat{\mathbf{s}}_{W-1}$.  The **per-pair
residual** for consecutive slot pair $(k-1, k)$ is:

$$
\mathbf{r}_k
  = \bigl(\hat{\mathbf{s}}_k - \hat{\mathbf{s}}_{k-1}\bigr) - \boldsymbol{\delta}_k^{\mathrm{odom}}
  \in \mathbb{R}^3
$$

with the angular component wrapped to $(-\pi, \pi]$.

The **window-span residual** accumulates across the full window:

$$
r_{\mathrm{span}}
  = \bigl\|\bigl(\hat{\mathbf{s}}_{W-1} - \hat{\mathbf{s}}_0\bigr)_{xy}
           - \textstyle\sum_{k=1}^{W-1}\boldsymbol{\delta}_{k,xy}^{\mathrm{odom}}\bigr\|
$$

$$
d_{\mathrm{span}}
  = \bigl\|\textstyle\sum_{k=1}^{W-1}\boldsymbol{\delta}_{k,xy}^{\mathrm{odom}}\bigr\|
$$

### 14.4 Slip-K Estimator

When `SdfCurrentSlotOnly = true`, only the **newest** window slot carries
an SDF observation; all internal slots are anchored solely by motion and
corner priors, so the L-BFGS optimizer drives their residuals to near-zero
by construction.  Averaging over all $W-1$ pairs would bias the estimate
downward by a factor of $W-1$.  The slip-k estimator therefore uses
**only the newest consecutive pair**:

$$
\hat{k}_s \;\leftarrow\; (1-\alpha)\,\hat{k}_s
  + \alpha \cdot \frac{|r_{\theta,W-1}|}{|\omega_{W-1}|}
$$

gated on $|\omega_{W-1}| \ge \omega_{\min}$ (default 0.05 rad/s) to
avoid division by near-zero angular speed.

### 14.5 Trans-Noise Estimator (Window Span, Variance-Based)

#### Why window span?

A single consecutive pair in one 55 ms slot covers only 0.02–0.05 m of
travel.  This barely clears the minimum-translation gate and produces
high fractional noise.  Accumulating over $W-1 \approx 4$ slots
(≈ 220 ms, ≈ 0.08–0.20 m) gives 4× larger signal for the same
sensor-noise floor.

#### Rotation gate

During turns the SDF has poor lateral position constraint, so
$\hat{\mathbf{s}}_{W-1} - \hat{\mathbf{s}}_0$ contains large
localization uncertainty unrelated to odometry error.  Samples are
skipped when the total window rotation satisfies:

$$
\left|\sum_{k=1}^{W-1} \delta_{\theta,k}^{\mathrm{odom}}\right| > 0.15\;\mathrm{rad}
$$

#### Variance sample and median buffer

A **variance sample** (not mean-absolute) is accumulated each qualifying
frame:

$$
v = \left(\frac{r_{\mathrm{span}}}{d_{\mathrm{span}}}\right)^2
$$

Samples are collected in a rolling buffer of size 10.  When the buffer
is full, a single EMA update is performed and the buffer is cleared:

$$
\hat{\sigma}_T \;\leftarrow\; (1-\alpha)\,\hat{\sigma}_T
  + \alpha\,\sqrt{\mathrm{median}(v_1,\ldots,v_{10})}
$$

> **Estimator unbiasedness.**  The mean-absolute estimator
> $E[|r|/d] = \sigma\sqrt{2/\pi} \approx 0.80\,\sigma$
> would be biased 25% low for Gaussian innovations.  Storing $(r/d)^2$
> and taking the square root of the median yields an unbiased ML
> estimate of $\sigma$ under the Gaussian assumption.

The median (rather than mean) suppresses outlier variance samples caused
by occasional poor localization frames that pass the quality gate.

### 14.6 Bias Estimator

The systematic bias is updated slowly from the mean residual over all
good pairs in the window:

$$
\hat{\mathbf{b}} \;\leftarrow\; (1-\beta)\,\hat{\mathbf{b}} + \beta\,\bar{\mathbf{r}}
$$

where $\bar{\mathbf{r}}$ is the mean of $\mathbf{r}_k$ over pairs whose
both adjacent slots pass the quality gate.  The slow rate
$\beta \ll \alpha$ prevents single-frame pose jumps from biasing the
accumulator.

The learned bias is currently **diagnostic only** and is not subtracted
from `odom_delta_tensor` during covariance computation.  It quantifies
consistent systematic drift that future work could subtract as a
feedforward correction.

### 14.7 Warmup and Override

- For the first `MotionLearnMinFrames` frames (default 50, ≈ 10 s at 5 Hz),
  the EMA estimators accumulate but `compute_motion_covariance()` continues
  using the static **`EncoderRotSlipK`** and **`OdomNoiseTrans`** from config.
- After warmup, it switches to the live learned values for measured
  odometry slots.  The command-prior slots are unaffected, providing a
  stable fallback at all times.
- Learned state resets on every `RoomConcept` re-initialisation (recovery
  event), restarting the warmup period.

### 14.8 Configuration

```toml
[RoomConcept]
LearnMotionModel            = true    # master switch (false by default)
MotionLearnAlpha            = 0.05    # EMA rate for slip-k and trans-noise
MotionLearnBeta             = 0.02    # EMA rate for bias (slower)
MotionLearnMinOmega         = 0.05    # rad/s — min angular speed for slip-k sample
MotionLearnMinTrans         = 0.02    # m — min accumulated window-span translation
MotionLearnQualityThreshold = 0.12    # SDF-MSE gate (more permissive than Hessian gate)
MotionLearnMinFrames        = 50      # warmup frames before learned values are active
```

### 14.9 Debug Output

Every 50 frames the learner prints a diagnostic line:

```
[MotionLearn] frames=N  slip_k=X  trans_noise=Y  bias=[bx,by,bt]
  (slip_pairs=P  trans_s=S  trans_buf=B  n_pairs=N  mse_span=old/new  gate=G)
```

| Field | Meaning |
|-------|---------|
| `slip_pairs` | Number of pairs that fired the slip-k update this report |
| `trans_s` | 1 if a trans-noise variance sample was collected this call |
| `trans_buf` | Current fill of the 10-sample median buffer |
| `mse_span` | SDF MSE at oldest / newest window slot |
| `gate` | Quality threshold in use |

### 14.10 Design Notes

The current implementation was reached after diagnosing five successive
bugs, all of which independently drove `trans_noise` toward unrealistically
low values:

| Bug | Root cause | Fix |
|-----|-----------|-----|
| Multi-pair averaging | Internal pairs have $r_k \approx 0$ under `SdfCurrentSlotOnly`; averaging all $W-1$ pairs gave true noise / $(W-1)$ | Newest-pair-only for slip-k; window-span for trans-noise |
| Learning starvation | `boundary_hessian_quality_threshold = 0.08` rejected nearly every moving frame | Separate `motion_learn_quality_threshold = 0.12` |
| Low sample rate | Single 55 ms pair below `min_trans` most frames | Window-span accumulation over ~220 ms |
| Rotation-induced noise | During turns, SDF lateral uncertainty dominates span residual | Rotation gate at 0.15 rad total window rotation |
| Biased estimator | Mean-absolute $E[|r|/d] = 0.80\,\sigma$ (25% low) | Variance samples $(r/d)^2$; flush with $\sqrt{\mathrm{median}}$ |
