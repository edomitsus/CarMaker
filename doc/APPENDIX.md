# Appendix: MPPI Formulation and Supporting Methods

This appendix gives the full mathematical formulation of the steering-only MPPI
controller implemented in [`src/User.cpp`](../src/User.cpp), including every
piece of supporting logic (engagement gating, the hard safety guardrail, and
curvature-aware obstacle localization) that the core algorithm depends on.
Equations are written in standard LaTeX math delimiters (`$...$` inline,
`$$...$$` display) so this file can be pasted directly into a LaTeX/Beamer
document, or rendered as-is in any Markdown viewer with math support (GitHub,
Notion, Typora, Obsidian, VS Code preview with a math extension).

All constants below are the actual values used in the implementation, not
illustrative placeholders.

---

## A. Notation and coordinate frames

| Symbol | Meaning |
|---|---|
| $s$ | Arc-length distance along the road route (road-frame longitudinal coordinate) |
| $t$ | Lateral offset from the route/lane center line (road-frame lateral coordinate); positive = left |
| $\psi$ | Vehicle heading relative to the local road tangent (0 = pointed exactly down the road) |
| $v$ | Vehicle speed $[\mathrm{m/s}]$ |
| $\delta_b$ | IPGDriver's own baseline steering-wheel angle for the current cycle $[\mathrm{rad}]$ |
| $\delta_m$ | MPPI's own commanded steering-wheel angle (the *delta*, not an absolute command) $[\mathrm{rad}]$ |
| $\kappa(s,t)$ | Road curvature in the $(x,y)$-plane at road-frame point $(s,t)$, from `RoadRouteEval` $[\mathrm{1/m}]$ |

Three distinct reference frames appear in the code and are converted between
explicitly:

1. **Object Sensor frame** — $(dx, dy)$, vehicle-heading-aligned, as reported
   directly by the Object Sensor (`ObjectSensor_GetObjectByObjId`).
2. **Road frame** — $(s,t)$, arc length and lateral offset along the route;
   this is the frame MPPI's entire cost function and rollout reason in.
3. **World frame** — global $(x,y)$ (CarMaker's `Fr0`), used only for
   plotting/visualization and for the curvature-aware obstacle localization
   round-trip described in §J.

---

## B. Control architecture: baseline + delta

The executed steering command is never MPPI's raw output. It is IPGDriver's
own road-following baseline (unrestricted authority, so it alone can execute
turns sharper than MPPI's own steering limit) plus a small bounded delta from
MPPI:

$$
\delta_{\text{applied}} \;=\; \delta_b \;+\; \delta_m, \qquad
|\delta_m| \le \delta_{\max}^{\text{MPPI}} = 15^\circ
$$

MPPI's own sampled control **is** $\delta_m$ directly — the rollout's
kinematic model adds $\delta_b$ back in only to evaluate what the vehicle
would actually do (§D), but the optimization variable and the quantity
returned to the caller are both the delta alone.

---

## C. Engagement gating

The full $256\times 60$ rollout only runs when a boolean gate fires; otherwise
$\delta_m \equiv 0$ and IPGDriver's baseline passes straight through
untouched (see §I2 for what "untouched" now precisely means).

**Gate condition** (logical OR of two independent checks):

$$
\text{Engage} \;=\; \text{CollisionRisk}(v) \;\lor\; \big(\text{RecoveringFromObstacle} \land |t| > t_{\text{tol}}\big)
$$

with $t_{\text{tol}} = 0.3\,\mathrm{m}$, and `RecoveringFromObstacle` a
persistent flag: set the instant `CollisionRisk` first fires, cleared the
instant $|t|$ returns under $t_{\text{tol}}$. This means MPPI can only ever
engage because of an actual sensed obstacle, or while still returning to
center from one — never on lateral offset alone, and never merely because
the vehicle is close to the road edge.

### C.1 Collision-risk sensing

For every currently tracked obstacle $i$ with sensor-frame offset
$(dx_i, dy_i)$ and radius $r_i$:

**Fixed geometric danger zone** (any speed):

$$
D_i^2 \;=\; \left(\frac{dx_i}{R_{\text{long}}}\right)^{\!2} + \left(\frac{dy_i}{r_i}\right)^{\!2},
\qquad R_{\text{long}} = 25\,\mathrm{m}
$$

$$
\text{triggers if } D_i^2 < 1
$$

**Speed-aware time-to-collision**, gated to the actual lane corridor in
road-frame (not sensor-frame — see §J for why this matters on a curve):

$$
\text{TTC}_i = \frac{dx_i}{v}, \qquad \text{triggers if } dx_i>0 \;\land\; v>0.1 \;\land\; \text{TTC}_i < T_{\text{TTC}}=4.0\,\mathrm{s}
$$

$$
\text{AND } |t_i^{\text{road}}| < \big(\text{corridor half-width in that direction}\big) + 2.0\,\mathrm{m}
$$

where $t_i^{\text{road}}$ is the obstacle's road-frame lateral position from
§J. The corridor gate is what prevents an object that is merely close in a
straight-line sensor reading — but actually around a sharp corner, on a
different road segment entirely — from falsely triggering engagement.

---

## D. Prediction model — kinematic bicycle, curvature-aware

Each of the $N=256$ samples rolls a kinematic bicycle model forward over
$H=60$ steps at $\Delta t = 0.05\,\mathrm{s}$ (a 3.0 s horizon):

$$
\theta_k \;=\; \frac{\delta_b + \delta_m^{(k)}}{\rho}, \qquad \rho = 16 \text{ (steering-wheel : road-wheel ratio)}
$$

$$
\Delta s_k \;=\; v\cos(\psi_k)\,\Delta t
$$

$$
s_{k+1} = s_k + \Delta s_k, \qquad
t_{k+1} = t_k + v\sin(\psi_k)\,\Delta t
$$

$$
\psi_{k+1} \;=\; \operatorname{wrap}\!\left(\psi_k + \frac{v}{L}\tan(\theta_k)\,\Delta t \;-\; \kappa(s_k,t_k)\,\Delta s_k\right), \qquad L = 2.8\,\mathrm{m}
$$

The $-\kappa(s_k,t_k)\,\Delta s_k$ term is what makes this **curvature-aware**:
without it, $\psi$ is tracked relative to a single road-heading snapshot taken
at $k=0$, silently assuming the road stays straight for the entire horizon —
wrong by an increasing amount the further ahead a 3 s/~60 m horizon looks on
an actual curve. Reading the road's own curvature at the predicted point and
bending the heading reference to match keeps the rollout physically
consistent with where the road actually goes.

Baseline steering $\delta_b$ is held **constant** across the horizon (a fair
approximation on a constant-or-slowly-varying-radius curve) — only $\delta_m$
varies per sample.

---

## E. Sampling: AR(1)-correlated exploration noise

Independent per-step Gaussian noise produces physically implausible steering
chatter (a real steering wheel can't teleport between angles frame to frame),
so noise is generated as a first-order autoregressive process:

$$
\epsilon_k \;=\; \alpha\,\epsilon_{k-1} + \sqrt{1-\alpha^2}\;\xi_k, \qquad \xi_k \sim \mathcal N(0,\sigma^2)
$$

$$
\alpha = 0.70 \text{ (MppiNoiseCorrelation)}, \qquad \sigma = 4^\circ \text{ (MppiNoiseStd)}
$$

The sampled control at each step is the current nominal sequence plus this
correlated noise, clamped to MPPI's own authority:

$$
\delta_m^{(k)} = \operatorname{clamp}\!\big(u_k + \epsilon_k,\; -15^\circ,\; 15^\circ\big)
$$

where $u_k$ is the running **nominal control** sequence (warm-started from the
previous cycle's solution, receding-horizon style — see §G).

---

## F. Cost function

Total cost per sample is the horizon integral of a running cost plus an
information-theoretic exploration term, plus a terminal cost:

$$
J = \sum_{k=0}^{H-1} \Delta t\Big(\ell(s_k,t_k,\psi_k,\delta_m^{(k)}) + \lambda\, \frac{u_k\,\epsilon_k}{\sigma^2}\Big) \;+\; \Phi(s_H,t_H,\psi_H)
$$

with temperature $\lambda = 5.0$ (MppiTemperature).

### F.1 Running cost $\ell$

$$
\ell = \underbrace{w_t\, t_k^2}_{\text{lane-center (=0, IPGDriver's job)}}
+ 60\,\psi_k^2
+ 0.2\,(\delta_m^{(k)})^2
+ 0.5\,(\delta_m^{(k)}-\delta_m^{(k-1)})^2
+ C_{\text{obs}}(s_k,t_k) + C_{\text{road}}(s_k,t_k)
$$

$w_t = 1.0$ (`LaneCenterWeight`) — deliberately small since centering is
primarily IPGDriver's responsibility; MPPI only needs enough pull to keep its
*own* delta from drifting arbitrarily once engaged.

### F.2 Terminal cost $\Phi$

$$
\Phi = 3.0\, t_H^2 + 100\,\psi_H^2 + C_{\text{obs}}(s_H,t_H) + C_{\text{road}}(s_H,t_H)
$$

### F.3 Obstacle cost $C_{\text{obs}}$ — smooth Gaussian + penetration + passing preference

For each tracked obstacle $i$ at road-frame position $(s_i, t_i)$ (from §J):

$$
D_i^2 = \left(\frac{s-s_i}{25}\right)^{\!2} + \left(\frac{t-t_i}{r_i}\right)^{\!2}
$$

$$
C_{\text{obs},i} = \underbrace{250\,e^{-\frac12 D_i^2}}_{\text{smooth avoidance pressure}}
\;+\; \underbrace{3.0\; g_{\text{long}}\, g_{\text{lat}}\,(t-2.25)^2}_{\text{left-pass preference, proximity-gated}}
\;+\; \underbrace{\mathbb 1[D_i^2<1]\big(2000 + 8000\,(1-D_i^2)^2\big)}_{\text{penetration penalty}}
$$

where the passing-preference gates are
$g_{\text{long}} = e^{-\frac12(\frac{s-s_i}{25})^2}$ and
$g_{\text{lat}} = e^{-\frac12(\frac{t-t_i}{r_i})^2}$ — both reuse terms
already computed above, so the $2.25\,\mathrm m$ left-pass pull fades out for
objects that are near in one axis but far in the other (e.g. a car 30 m off
to the side), not just far in Euclidean terms.
$C_{\text{obs}} = \sum_i C_{\text{obs},i}$.

### F.4 Road boundary cost $C_{\text{road}}$ — deliberately asymmetric

$$
C_{\text{road}}(t) =
\begin{cases}
w_s^{\pm}(|t|-h^{\pm}+1.0)^2 & \text{if } |t| > h^{\pm}-1.0 \\[2pt]
+\; w_h^{\pm}\big(1+(|t|-h^{\pm})^2\big) & \text{if } |t| > h^{\pm}
\end{cases}
$$

where $h^\pm$ (`CurrentLeftRoadLimit` / `CurrentRightRoadLimit`) is a
per-cycle snapshot of the current lane's half-width (right) or half-width
plus adjacent lane width (left), and the weights differ by roughly **1000×**
between sides:

| | soft weight $w_s$ | hard weight $w_h$ |
|---|---|---|
| Left (passing lane — crossing briefly is fine) | 3,000 | $1\times10^6$ |
| Right (true pavement edge — nothing recoverable past it) | 30,000 | $1\times10^9$ |

$h^\pm$ is deliberately **not** re-evaluated per rollout step against
`RoadRouteEval`'s route-relative width (that frame differs from
`Vehicle.Road.Path.tRoad`'s lane-relative frame by roughly half a lane width —
mixing the two was an earlier, since-fixed bug where the guardrail fired at
the wrong distance). It is additionally floor-checked each cycle
(`MinimumPlausibleActWidth = 2.5\,\mathrm m`): a junction/connector link can
report an implausibly narrow placeholder width, and a reading below the floor
holds the last plausible snapshot rather than collapsing the boundary to it.

---

## G. MPPI update law — information-theoretic weighting

After scoring all $N$ samples, the nominal control sequence is updated via
the standard path-integral (Boltzmann/softmax) weighting, **not** a simple
"take the best sample":

$$
w^{(i)} = \exp\!\left(-\frac{J^{(i)} - J_{\min}}{\lambda}\right), \qquad
u_k \leftarrow \operatorname{clamp}\!\left(u_k + \frac{\sum_i w^{(i)}\,\epsilon_k^{(i)}}{\sum_i w^{(i)}},\; -15^\circ,\;15^\circ\right)
$$

Samples with $\frac{J^{(i)}-J_{\min}}{\lambda} \ge 60$ are given zero weight
outright (numerically negligible, and this is what makes a genuine road-edge
or obstacle-penetration cost — scaled into the tens of thousands — an
effectively *hard* constraint: no on-road sample's weight can be outweighed by
one that crosses the line, regardless of what obstacle cost it saved).

**Receding horizon shift**: after the update, $u_0$ is returned as the
command and the sequence is shifted, $u_k \leftarrow u_{k+1}$, with the final
slot repeating the last value — standard MPC warm-starting so each cycle
refines the previous plan instead of solving from scratch.

**Validity**: if $\sum_i w^{(i)}$ is non-finite or $\le 10^{-12}$ (numerical
degeneracy — e.g. every sample scored $+\infty$), the result is marked
invalid and the caller falls back to baseline alone rather than trusting an
ill-defined update.

### G.1 Delta authority taper (anti-stacking fix)

A late addition: MPPI's delta is scaled down as $|\delta_b|$ grows, so it
cannot stack unchecked on top of a baseline that is already committing
serious steering angle to execute a turn on its own:

$$
\delta_m^{\text{applied}} = \delta_m \cdot \sigma(\delta_b), \qquad
\sigma(\delta_b) = \operatorname{clamp}\!\left(\frac{40^\circ - |\delta_b|}{40^\circ-20^\circ},\;0,\;1\right)
$$

Full authority below $20^\circ$ baseline magnitude, linearly tapering to zero
by $40^\circ$ — smooth rather than a cliff-edge, consistent with every other
cost term in this file.

---

## H. Failure-mode handling

- **Invalid MPPI result** → command falls back to $\delta_b$ alone (not a
  standalone absolute correction — dropping baseline entirely was observed to
  run the vehicle off a curving road).
- **Stall recovery**: at low speed after having previously moved (a stall,
  not startup), a *one-time* snapshot of $\delta_b$ is taken at stall onset
  and held, rather than continuously re-reading it. Continuous re-reading
  turned a fixed nonzero correction into a literal unbounded discrete
  integrator in earlier testing (`DrivMan.Steering.Ang` is written by this
  same function every cycle, so re-reading it while adding a constant
  correction is a feedback loop, observed to climb past $600^\circ$ over a
  30 s stall). The freeze is now applied *only* while a nonzero recovery
  correction is actually being added (that's the specific thing that
  integrates) — with no obstacle memory to recover from, the live baseline is
  tracked directly instead, so IPGDriver's own steering law isn't needlessly
  frozen for the whole stall duration.

---

## I. Hard safety guardrail

The cost function above is a strong *preference*, not a guarantee — if no
sampled trajectory both clears an obstacle and stays on the road, the
optimizer still returns whichever sample was least bad, which could itself be
off-road. A second, independent, deterministic layer sits underneath it.

### I.1 Proportional trigger (not bang-bang)

$$
\delta_{\text{guard}} =
\begin{cases}
-\delta_{\max}^{\text{guard}}\cdot \operatorname{clamp}\!\left(\dfrac{t - (h^{L}-0.15)}{1.0},\,0,\,1\right) & t > h^{L}-0.15 \\[6pt]
+\delta_{\max}^{\text{guard}}\cdot \operatorname{clamp}\!\left(\dfrac{-(h^{R}-0.15) - t}{1.0},\,0,\,1\right) & t < -(h^{R}-0.15)
\end{cases}
$$

with $\delta_{\max}^{\text{guard}} = 45^\circ$ (`EdgeGuardrailSteeringLimit`)
and a $1.0\,\mathrm m$ ramp distance. This overrides $\delta_{\text{applied}}$
entirely (not just $\delta_m$) — no cost function, no sampling, nothing an
obstacle can outweigh.

Two tuning lessons are baked into these specific values:

- $\delta_{\max}^{\text{guard}}$ must be **at least** as large as ordinary
  baseline steering during a normal turn — reusing MPPI's own $15^\circ$
  authority here meant the "last resort" could be *weaker* than what was
  already failing.
- The override must be **proportional**, not instant-to-maximum: a bang-bang
  jump straight to full authority the instant the trigger margin is crossed
  was observed to overshoot back through lane center and off the *opposite*
  edge during a tight, already-mid-correction obstacle pass. Ramping from
  zero at the threshold avoids that while still reaching full authority if
  the excursion keeps getting worse.

### I.2 Scope: obstacle episodes only, and true zero-interference otherwise

Both the guardrail and MPPI's own reconvergence engagement are restricted to
firing only while `RecoveringFromObstacle` is true (§C) — a road departure
with no obstacle behind it is left entirely to IPGDriver, by explicit design
choice, not merely by omission.

When nothing is actively correcting, the executed command bypasses this
file's own rate/acceleration limiter entirely:

$$
\delta_{\text{applied}} =
\begin{cases}
\operatorname{clamp}\!\big(\delta_{\text{target}} - \delta_{\text{prev}},\, -\dot\delta_{\max}\Delta t,\, \dot\delta_{\max}\Delta t\big) + \delta_{\text{prev}} & \text{actively correcting} \\
\delta_b & \text{otherwise (true passthrough)}
\end{cases}
$$

with $\dot\delta_{\max}=30^\circ/\mathrm s$. IPGDriver has its own internal
steering-actuator dynamics ($630^\circ$/s velocity limit, $3000^\circ/\mathrm
s^2$ acceleration limit in its own configuration) — applying a second, ~16×
slower rate limiter on top even while contributing zero correction was a real
source of phase lag between what baseline wanted and what was actually sent,
compounding into real trajectory error over a fast maneuver despite looking
identical at sparse logged snapshots.

---

## J. Curvature-aware obstacle localization

Converting an obstacle's sensor-frame $(dx,dy)$ into road-frame $(s_i,t_i)$
matters for every equation above that uses $(s_i,t_i)$ — §C.1's corridor
gate, and §F.3's cost anchor.

**First-order estimate** (single rotation by the vehicle's current
heading-relative-to-road $\psi$, assuming the road stays straight for the
whole distance $dx$ ahead):

$$
s_i \approx s_{\text{ref}} + dx\cos\psi - dy\sin\psi, \qquad
t_i \approx t_{\text{ref}} + dx\sin\psi + dy\cos\psi
$$

Correct on a straight road; increasingly wrong the more the road curves over
distance $dx$ (which can be tens of meters — $\text{TTC}$ gating alone allows
up to $v\cdot 4\,\mathrm s$). On a road curving *toward* the obstacle, this
underestimates how far into the lane corridor the obstacle already is, which
was the direct cause of avoidance starting too late on curved roads (and,
consequently, needing a sharper corrective swerve afterward).

**Refinement**: rotate by the vehicle's true *world* heading $\Psi$ (not
heading-relative-to-road) to get the obstacle's actual world position, then
resolve that exact point against the real route geometry via
`RoadRouteEval(..., RIT_XY_S, ...)` — CarMaker's inverse query, world
$(x,y)\to$ route $(s,t)$:

$$
x_i^{\text{world}} = x^{\text{world}} + dx\cos\Psi - dy\sin\Psi, \qquad
y_i^{\text{world}} = y^{\text{world}} + dx\sin\Psi + dy\cos\Psi
$$

$$
(s_i, t_i) = \texttt{RoadRouteEval}^{-1}\big(x_i^{\text{world}}, y_i^{\text{world}}\big)
$$

This follows the actual curve between vehicle and obstacle exactly, however
sharply it bends, rather than assuming it doesn't — the first-order estimate
above is retained only as the search hint / fallback if the road-eval handle
isn't ready.

---

## K. Complete parameter reference

| Symbol / name | Value | Meaning |
|---|---|---|
| $N$ (`MppiSamples`) | 256 | Rollout samples per update |
| $H$ (`MppiHorizon`) | 60 | Steps per rollout |
| $\Delta t$ (`MppiStep`) | 0.05 s | Step size (3.0 s horizon) |
| Update period | 0.05 s | Controller runs at 20 Hz |
| $\lambda$ (`MppiTemperature`) | 5.0 | Softmax temperature |
| $\sigma$ (`MppiNoiseStd`) | $4^\circ$ | Exploration noise std. dev. |
| $\alpha$ (`MppiNoiseCorrelation`) | 0.70 | AR(1) correlation |
| $L$ (`MppiWheelbase`) | 2.8 m | Bicycle-model wheelbase |
| $\rho$ (`MppiSteeringRatio`) | 16.0 | Steering-wheel : road-wheel ratio (own kinematic model only) |
| $\delta_{\max}^{\text{MPPI}}$ (`SteeringLimit`) | $15^\circ$ | MPPI's own delta authority |
| $\dot\delta_{\max}$ (`SteeringRateLimit`) | $30^\circ/\mathrm s$ | Rate limit on our own correction only |
| $\ddot\delta_{\max}$ (`SteeringAccelerationLimit`) | $2\,\mathrm{rad/s^2}$ | Acceleration limit, same scope |
| Recovery correction limit | $5^\circ$ | Cap on stall-recovery nudge |
| Absolute command ceiling | $720^\circ$ | Defensive backstop only |
| $R_{\text{long}}$ (`ObstacleLongitudinalRadius`) | 25 m | Obstacle envelope, longitudinal |
| Obstacle lateral radius | 1.85 m | Obstacle envelope, lateral |
| Preferred passing offset | 2.25 m | Deterministic left-pass target |
| $T_{\text{TTC}}$ | 4.0 s | Time-to-collision trigger threshold |
| Corridor margin | 2.0 m | TTC gate corridor slack |
| Reconvergence tolerance | 0.3 m | "Back to center" threshold |
| Road safety margin | 1.0 m | Soft-limit standoff from hard limit |
| Left soft/hard weight | 3,000 / $10^6$ | Passing-lane boundary |
| Right soft/hard weight | 30,000 / $10^9$ | True pavement edge (≈1000× left) |
| Default road width | 6.0 m | Conservative eval fallback |
| Min. plausible width | 2.5 m | Junction-artifact floor |
| Guardrail margin | 0.15 m | Trigger distance inside true edge |
| Guardrail authority | $45^\circ$ | Independent of MPPI's own limit |
| Guardrail ramp distance | 1.0 m | Proportional, not bang-bang |
| Delta-taper start / end | $20^\circ$ / $40^\circ$ | MPPI authority fades out over this baseline range |

