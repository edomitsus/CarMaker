# CarMaker Steering-Only MPPI Controller

A Model Predictive Path Integral (MPPI) controller for obstacle avoidance in CarMaker Office,
layered on top of IPGDriver's own road-following rather than replacing it. MPPI only computes
a small steering correction when it actually senses a need to — otherwise the vehicle drives
exactly as IPGDriver's baseline would.

All of the controller logic lives in [`src/User.cpp`](src/User.cpp).

---

## Demos

<!--
  Drop your video files into the media/ folder using these exact names, or update the
  src="" paths below to match whatever you name them. GitHub renders committed mp4/webm/mov
  files as an inline player automatically - no extra markup needed beyond the <video> tag.
-->

### MPPI avoiding an obstacle

<video src="media/default_view_web.mp4" controls width="720"></video>

### 2D visualization — sampled paths colored by cost

Every path MPPI considered on a given update, colored by rank so the cheapest (chosen) path
stands out, replayed frame by frame as the car moves. Produced with
[`tools/mppi_rollout_viewer.html`](tools/mppi_rollout_viewer.html).

<video src="media/2DMPPI_world.mp4" controls width="720"></video>
<video src="media/2DMPPI_road.mp4" controls width="720"></video>

### 3D visualization — sampled paths overlaid on real footage

The same sampled paths projected onto real MovieNX-recorded footage from a calibrated,
vehicle-fixed camera model, so you can see what MPPI was evaluating superimposed on the
actual simulation video. Produced with [`mppi_video_overlay.html`](mppi_video_overlay.html).

<video src="media/3DMPPI.mp4" controls width="720"></video>

### Simulator footage

Raw CarMaker/MovieNX recording of the scenario, no overlay.

<video src="media/UsT3R.mp4" controls width="720"></video>

---

## Method summary

### Overview

The controller entry point is `User_DrivMan_Calc()`. Each cycle it captures IPGDriver's own
steering command for that cycle as a **baseline** *before* touching anything, then decides
whether MPPI should add a correction on top of it:

```
final steering = ipgBaselineSteering + mppiDelta
```

The baseline has full, unrestricted steering authority — it's what actually tracks the
route/lane center and handles curves, including ones sharp enough to need more angle than
MPPI itself is allowed to command. MPPI's own output is a bounded delta (±15°) meant only to
nudge the vehicle around something in the way, not to drive the car by itself.

### When MPPI engages

MPPI does not run continuously — running the full 256-sample rollout every cycle when nothing
is happening would be wasted computation, and worse, an active-but-unneeded controller is one
more thing that can go subtly wrong. `NeedsMppiEngagement()` gates it on two conditions, either
of which is enough:

- **A collision risk is sensed** (`AnyCollisionRiskSensed()`): a fixed geometric danger-zone
  check around each tracked obstacle, plus a speed-aware time-to-collision check so the
  trigger distance grows with speed instead of giving less and less warning time the faster
  the vehicle goes. The time-to-collision branch is also **corridor-gated**: an obstacle's
  position is rotated into the road frame and only counts as a real threat if it actually
  falls within the current lane's width (plus a small margin) — this is what stops MPPI from
  falsely engaging for an object that's simply on the road *after* a sharp corner, which would
  otherwise look deceptively close in a straight-line sensor reading.
- **The vehicle hasn't reconverged to lane center** after a maneuver — kept active a little
  longer than the collision risk alone would justify, so it can help the baseline finish
  recovering on a curve instead of handing off a real lateral error mid-turn.

### Prediction model

`SteeringMppi::Update()` rolls out a kinematic bicycle model over a 3-second, 60-step horizon
at 20 Hz, sampling 256 candidate steering-delta sequences per update. Each rollout tracks
`(roadDistance, lateralPosition, heading)` forward in time using the candidate steering plus
the baseline held constant across the horizon.

The rollout is **curvature-aware**: it queries the road's actual curvature at each predicted
point (`RoadRouteEval`) and bends the heading reference accordingly, instead of assuming the
road stays pointed in whatever direction it happened to face at the start of the horizon. On a
tight curve, a rollout that didn't do this would silently become wrong within a second or two
of prediction.

### Sampling and optimization

Candidate steering sequences are generated as AR(1)-correlated Gaussian noise added to a
running nominal control sequence (not independent noise per step, which would produce
physically implausible steering chatter). After scoring every sample's rollout, the nominal
sequence is updated via the standard MPPI softmax/Boltzmann weighting:

```
weight[i] = exp(-(cost[i] - min(cost)) / temperature)
```

lower-cost samples pull the nominal sequence toward themselves more strongly. This is
information-theoretic model-predictive control, not just "pick the best sample" — the update
is a weighted blend across the whole batch.

### Cost function

- **Obstacle cost** — a smooth Gaussian penalty around each tracked obstacle (separate
  longitudinal/lateral radii), with a steep additional penalty once a rollout actually
  penetrates the obstacle's envelope. A deterministic left-passing preference nudges ties
  toward passing on the left, but is itself gated by proximity so it doesn't pull the vehicle
  toward far-off objects that pose no real risk. Obstacle positions are converted from the
  Object Sensor's vehicle-heading-aligned sensor frame into the road frame by rotating through
  the vehicle's current heading — necessary because the two frames only coincide when the
  vehicle is pointed exactly down the road, which isn't true mid-maneuver or on a curve.
- **Road boundary cost** — deliberately **asymmetric**. The right edge is the real pavement
  edge with nothing recoverable past it, so it carries a penalty roughly 1000x steeper than
  the left edge, which is the boundary into a passing lane — crossing it briefly to get around
  a slower car is normal, intended behavior. Both limits come from CarMaker's live lane-width
  query (`Vehicle.Road.Act` / `Vehicle.Road.OnLeft`) rather than an assumed fixed road width.
- **Lane-center pull** — a modest running and terminal cost keeping MPPI's own delta anchored
  near center once engaged. Kept deliberately small since centering itself is primarily the
  baseline's job.

### Hard safety guardrail

The cost function above is a strong *preference*, not a guarantee — if literally no sampled
trajectory manages to both clear an obstacle and stay on the road, the optimizer still returns
whichever sample was least bad, which could itself be off-road. Underneath the cost-based
system sits a second, independent layer: the instant the vehicle's *actual* position gets
within a small margin of the true road edge, steering is deterministically overridden to
recover toward center — no cost function, no sampling, nothing that can be outweighed by an
obstacle. This should almost never fire if the tuning above is doing its job; it's the
guarantee underneath the preference.

### Failure-mode handling

- If MPPI's result is invalid (e.g. numerical degeneracy in the softmax weights), steering
  falls back to the baseline alone rather than an unrestricted absolute correction.
- At low speed after the vehicle has previously been moving (a stall, not startup), steering
  enters a gentle **recovery** mode using a *one-time* snapshot of the baseline rather than
  continuously re-reading it — re-reading it every cycle while stalled turned a fixed nonzero
  correction into a literal unbounded discrete integrator in earlier testing, which is exactly
  the kind of bug a one-line snapshot fix avoids for good.

### Visualization tooling

Two browser-based tools (self-contained HTML, no build step) consume a CSV that
`SteeringMppi::AppendRolloutFrame()` writes while MPPI is engaged — every sampled path, its
cost, the road shape, and any tracked obstacles, captured periodically so a whole maneuver can
be replayed rather than seeing only a single snapshot:

- **[`tools/mppi_rollout_viewer.html`](tools/mppi_rollout_viewer.html)** — top-down playback,
  either in real-world shape (matches the curvature you'd actually see driving) or MPPI's own
  road-relative frame (distance along the road vs. lateral offset, useful for reading error
  directly even though the road renders as a flat band).
- **[`mppi_video_overlay.html`](mppi_video_overlay.html)** — projects the same sampled paths
  onto real recorded footage from a calibrated, vehicle-fixed perspective camera, so the
  candidate paths can be seen superimposed on what the simulation actually looked like.
