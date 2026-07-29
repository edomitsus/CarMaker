/*
 *****************************************************************************
 *  CarMaker - Version 15.0
 *  Virtual Test Driving Tool
 *
 *  Copyright ©1998-2025 IPG Automotive GmbH. All rights reserved.
 *  www.ipg-automotive.com
 *****************************************************************************
 *
 * Functions
 * ---------
 *
 * Initialization
 *
 *	User_Init_First ()
 *	User_PrintUsage ()
 *	User_ScanCmdLine ()
 *
 *	User_AppLogFilter ()
 *
 *	User_Init ()
 *	User_Register ()
 *	User_DeclQuants ()
 *
 *	User_Param_Add ()
 *	User_Param_Get ()
 *
 *
 * Main Test Run Start/End:
 *
 *	User_TestRun_Start_atBegin ()
 *	User_TestRun_Start_atEnd ()
 *	User_TestRun_Start_StaticCond_Calc ()
 *	User_TestRun_Start_Finalize ()
 *	User_TestRun_RampUp ()
 *
 *	User_TestRun_End_First ()
 *	User_TestRun_End ()
 *
 *
 * Main Cycle:
 *
 *	User_In ()
 *
 *	User_DrivMan_Calc ()
 * 	User_Traffic_Calc ()
 *	User_VehicleControl_Calc ()
 *	User_Brake_Calc ()           in Vhcl_Calc ()
 *	User_Calc ()
 *	User_Check_IsIdle ()
 *
 *	User_Out ()
 *
 *
 * APO Communication:
 *
 *	User_ApoMsg_Eval ()
 *	User_ApoMsg_Send ()
 *
 *	User_ShutDown ()
 *	User_End ()
 *	User_Cleanup ()
 *
 *
 *****************************************************************************
 */

#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <array>
#include <cmath>
#include <limits>
#include <random>

#if defined(XENO)
# include <mio.h>
#endif
#include <ioconf.h>

#include <CarMaker.h>
# include <Car/Vehicle_Car.h>
#include <Road.h>
#include <Vehicle/Sensor_Object.h>

#include <ADASRP.h>


#include "IOVec.h"
#include "User.h"

/* @@PLUGIN-BEGIN-INCLUDE@@ - Automatically generated code - don't edit! */
/* @@PLUGIN-END@@ */

int UserCalcCalledByAppTestRunCalc = 0;

tUser User;

namespace {

constexpr double Pi = 3.14159265358979323846;

/* Object Sensor obstacle cost envelope. */
constexpr double ObstacleLongitudinalRadius = 25.0;       /* avoidance envelope [m] */
constexpr double ObstacleLateralRadius = 1.85;             /* avoidance envelope [m] */
constexpr double PreferredPassingLateralPosition = 2.25;  /* deterministic left pass [m] */
constexpr double RoadSafetyMargin = 1.0;                  /* edge approach margin [m] */
constexpr double DefaultRoadWidth = 6.0;                  /* conservative eval fallback [m] */
/* Junction/connector links in the road network can report a tiny placeholder
   Act.Width (observed: 1.0 m) that describes the network node itself, not
   the vehicle's actual navigable space while turning through the
   intersection. Trusting that reading collapsed CurrentRightRoadLimit/
   CurrentLeftRoadLimit to ~0.5 m mid-turn, which made the hard edge
   guardrail (and RoadBoundaryCost) fire against a nonsense boundary instead
   of the real one - see doc/PROJECT_CONTEXT.md "guardrail fires on junction
   geometry" task. No real single lane is this narrow, so a reading below
   this floor is treated as untrustworthy. */
constexpr double MinimumPlausibleActWidth = 2.5;          /* [m] */
/* Weights for RoadBoundaryCost() below (see doc/PROJECT_CONTEXT.md "road
   edge not punished enough" task). Old values (soft=500, hard=5000) were
   smaller than ObstacleCost()'s own max penetration cost (~10,250), so the
   optimizer wasn't actually deterred from grazing/lightly crossing the
   edge if doing so helped dodge something obstacle-related - the two
   costs were the wrong relative scale. RoadEdgeHardPenalty is deliberately
   enormous (same "effectively forbidden" pattern used elsewhere for
   collisions): scaledCost = (cost-min)/MppiTemperature will be far past
   the existing `scaledCost < 60` weight-survival cutoff, so any sample
   that actually crosses the edge gets exactly zero softmax weight
   whenever an on-road sample exists, regardless of what it saved on
   obstacle cost. */
constexpr double RoadEdgeSoftPenaltyWeight = 3000.0;
constexpr double RoadEdgeHardPenaltyWeight = 1.0e6;
/* FIX ("the right white line should be treated like a railing" task): the
   right edge of the current lane is the actual paved-road edge (grass/
   shoulder beyond it) - there is nothing recoverable past it, unlike the
   left edge, which in these scenarios is the boundary into the passing
   lane (crossing it briefly to get around a slower car is normal, intended
   behavior - see PreferredPassingLateralPosition). So the right side needs
   a categorically bigger deterrent than the left, not just the same "very
   large" cost: ~1000x the left hard weight, so that even in a genuine
   dilemma (car very close on one side, edge close on the other) the
   optimizer always prefers tightening up next to the car over crossing
   the right line. */
constexpr double RightRoadEdgeSoftPenaltyWeight = 30000.0;
constexpr double RightRoadEdgeHardPenaltyWeight = 1.0e9;

tRoadEval *MppiRoadEval = nullptr;
double RoadEvaluationSOffset = 0.0;
/* Current lane's own half-width (right limit) and half-width plus the
   adjacent lane's width if one exists (left limit), snapshotted once per
   outer cycle from Vehicle.Road.Act/OnLeft (see RoadBoundaryCost()
   and the edge guardrail below - "where is the right boundary" task).
   Deliberately NOT re-derived via RoadRouteEval() per rollout step: that
   API's width[] output is relative to the ROUTE's center line, but
   Vehicle.Road.Path.tRoad (and everything derived from it in the rollout)
   is relative to the PATH's/lane's own center - two different origins,
   offset by roughly half a lane width. Comparing a Route-relative border
   against a Path-relative position was the root cause of the guardrail
   firing at the wrong distance (~1.9 m instead of the lane's actual 2.75 m
   edge). Vehicle.Road.Act.Width/OnLeft.Width are already in the same
   Path-relative frame as tRoad, so no conversion is needed; the tradeoff is
   that they're a snapshot of the CURRENT position, held constant across
   the ~3-4 s rollout horizon rather than re-evaluated ahead - acceptable
   for these mostly-uniform road sections. */
double CurrentRightRoadLimit = 0.5 * DefaultRoadWidth;
double CurrentLeftRoadLimit = 0.5 * DefaultRoadWidth;

struct Obstacle {
    bool valid;
    double dx;
    double dy;
    double radius;
    int objId;
};

constexpr int MaxObstacles = 8;
std::array<Obstacle, MaxObstacles> SensorObstacles{};
int SensorObstacleCount = 0;
double SensorObstacleReferenceRoadDistance = 0.0;
double SensorObstacleReferenceLateralPosition = 0.0;
/* Ego heading-relative-to-road at the moment the two references above were
   captured (see ObstacleCost() below - "wrong side swerve" fix). Needed to
   rotate obstacle.dx/dy, which are in the vehicle's own heading-aligned
   SENSOR frame, into the road-aligned frame before combining them with
   road-frame reference values. */
double SensorObstacleReferenceHeading = 0.0;

double
NearestObstacleDx()
{
    return SensorObstacleCount > 0 ? SensorObstacles[0].dx : 999.0;
}

double
NearestObstacleDy()
{
    return SensorObstacleCount > 0 ? SensorObstacles[0].dy : 0.0;
}

/* FIX ("wrong side swerve" task): obstacle.dx/dy are in the vehicle's own
   heading-aligned SENSOR frame (Object Sensor NearPnt.ds), not the road
   frame - they must be rotated by the ego's heading-relative-to-road at
   capture time (SensorObstacleReferenceHeading) before being added onto the
   road-frame reference position. Previously this was a plain, un-rotated
   add, which is only correct when the vehicle points exactly down the road
   tangent; any heading offset (mid avoidance maneuver, or just a curve)
   shifted the obstacle's computed road-frame position, sometimes making the
   real gap look like it was on the wrong side. Factored out so both
   ObstacleCost() and AnyCollisionRiskSensed() (see its lateral corridor
   gate below) can report/reason about the same resolved position, and so
   the diagnostic log can show it too instead of only the raw sensor
   dx/dy. */
void
ObstacleAbsolutePosition(const Obstacle &obstacle, double &roadPosition, double &lateralPosition)
{
    const double cosReferenceHeading = cos(SensorObstacleReferenceHeading);
    const double sinReferenceHeading = sin(SensorObstacleReferenceHeading);
    const double obstacleRoadOffset =
        obstacle.dx * cosReferenceHeading - obstacle.dy * sinReferenceHeading;
    const double obstacleLateralOffset =
        obstacle.dx * sinReferenceHeading + obstacle.dy * cosReferenceHeading;
    roadPosition = SensorObstacleReferenceRoadDistance + obstacleRoadOffset;
    lateralPosition = SensorObstacleReferenceLateralPosition + obstacleLateralOffset;
}

constexpr double SteeringLimit = 15.0 * Pi / 180.0;      /* steering-wheel angle [rad] */
constexpr double SteeringRateLimit = 30.0 * Pi / 180.0;  /* [rad/s] */
constexpr double SteeringAccelerationLimit = 2.0;        /* [rad/s^2] */
constexpr double RecoverySteeringLimit = 5.0 * Pi / 180.0; /* gentle stalled recovery [rad] */
/* Defensive backstop only (see doc/PROJECT_CONTEXT.md "steering runaway
   while stalled" task): now that the executed command is baseline +
   correction rather than a value MPPI/fallback bounds by construction, add
   an explicit absolute ceiling so any unforeseen feedback path (the
   stalled-baseline fix above addresses the one found) can't drive the
   physical command to an absurd angle. Generous - about 2 full turns each
   way, well beyond any normal steering-wheel command this project issues. */
constexpr double MaxAbsoluteSteeringCommand = 720.0 * Pi / 180.0;

/* Hard edge guardrail (see User_DrivMan_Calc() below - "just don't want it
   to go over the edge" ask). RoadBoundaryCost() only makes crossing the
   edge EXPENSIVE, it doesn't make it impossible: it's one term in a
   weighted-average cost, so if literally no sampled maneuver this cycle
   both stays on the road and clears an obstacle, MPPI still returns
   whichever sample was least bad, which can itself be off-road. This is a
   deterministic backstop on the vehicle's ACTUAL current position (not a
   rollout guess): the instant tRoad gets within this margin of the real
   road edge, it overrides MPPI/baseline entirely and steers hard back
   toward center, no cost function involved, so it can't be outweighed by
   any obstacle. It's meant to almost never fire if the cost-based tuning
   above is doing its job - it's the guarantee underneath the preference. */
constexpr double RoadEdgeGuardrailMargin = 0.15; /* [m] trigger just inside the true edge */
/* Dedicated steering authority for the guardrail override, deliberately
   separate from MPPI's own SteeringLimit (+-15 deg, sized for a small
   avoidance nudge on top of baseline). Reusing SteeringLimit here meant the
   "last resort" override could be LESS authoritative than baseline's own
   ordinary steering during a normal turn (observed: baseline commanding
   -29.73 deg the instant before the guardrail took over and capped it down
   to -15 deg) - so the guardrail engaged in time but still couldn't arrest
   the excursion, being weaker than what was already failing. 45 deg
   (matching the largest baseline angles seen during normal curve-tracking
   elsewhere in these logs) is as far as this should go, though - a
   follow-up attempt at 90 deg was tried to fix a still-unrecovered sharp
   turn (baseline itself independently converging on ~-44.5 deg there and
   still failing), but it broke the ORIGINAL, previously-working obstacle
   pass instead: at a fixed EdgeGuardrailRampDistance, doubling the
   authority doubles the correction per meter of excess, and during a
   tight high-speed pass that's already mid-swerve, that stronger,
   faster-ramping correction overshot through lane center and out the
   OTHER side, crashing ~40s earlier than any previous run. More authority
   fights an already-occurring avoidance maneuver instead of just raising
   the ceiling for genuinely dire cases - back to 45 deg, which is the
   largest value observed not to cause that. The still-unresolved sharp
   turn needs a different fix than guardrail authority (see
   doc/PROJECT_CONTEXT.md "vehicle carries too much speed into a tight
   turn after a long stall" task). */
constexpr double EdgeGuardrailSteeringLimit = 45.0 * Pi / 180.0; /* [rad] */
/* FIX (observed: raising EdgeGuardrailSteeringLimit above fixed an
   undershoot on a sharp turn, but caused a much worse failure elsewhere -
   the guardrail used to snap straight to its full target the INSTANT
   tRoad crossed the trigger margin, bang-bang style. During a tight
   obstacle pass that's already mid-correction, that abrupt full-authority
   yank overshot back through lane center and swung the vehicle all the
   way into the OPPOSITE edge before the correction could reverse in time.
   A bigger authority just made that overshoot bigger in both directions -
   the bang-bang shape was the actual problem, not the magnitude. Ramping
   the override from 0 at the trigger margin up to full
   EdgeGuardrailSteeringLimit over this distance keeps it gentle right at
   the threshold (where a hard yank isn't yet warranted) while still
   reaching full authority if the excursion keeps getting worse. */
constexpr double EdgeGuardrailRampDistance = 1.0; /* [m] */

enum class SpeedState {
    Startup,
    Moving,
    Stalled
};

enum class ControllerMode {
    Fallback,
    Mppi,
    Recovery,
    Passthrough,  /* moving, no collision risk sensed: IPGDriver baseline only */
    EdgeGuardrail /* hard override: actual position too close to the road edge */
};

char const *
SpeedStateName(SpeedState state)
{
    switch (state) {
    case SpeedState::Moving:
        return "MOVING";
    case SpeedState::Stalled:
        return "STALLED";
    default:
        return "STARTUP";
    }
}

char const *
ControllerModeName(ControllerMode mode)
{
    switch (mode) {
    case ControllerMode::Mppi:
        return "MPPI";
    case ControllerMode::Recovery:
        return "RECOVERY";
    case ControllerMode::Passthrough:
        return "PASSTHROUGH";
    case ControllerMode::EdgeGuardrail:
        return "EDGE_GUARDRAIL";
    default:
        return "FALLBACK";
    }
}

constexpr int MppiSamples = 256;
constexpr int MppiHorizon = 60;
constexpr double MppiStep = 0.05;                        /* 3.0 s prediction horizon */
constexpr double MppiUpdatePeriod = 0.05;                /* 20 Hz controller update */
constexpr double MppiTemperature = 5.0;
constexpr double MppiNoiseStd = 4.0 * Pi / 180.0;
constexpr double MppiNoiseCorrelation = 0.70;
constexpr double MppiWheelbase = 2.8;                    /* bicycle-model wheelbase [m] */
constexpr double MppiSteeringRatio = 16.0;               /* steering wheel / road wheel */

/* Reduced, not zeroed (see doc/PROJECT_CONTEXT.md "separate road-following
   from obstacle avoidance" task): lane-centering is primarily IPGDriver's
   baseline's job now (added back in via SteeringMppi::Update()'s
   baselineSteering parameter), so this is far below the original 4.0/12.0
   to avoid MPPI's delta double-fighting a baseline that's already doing
   the job. But fully zeroing it (tried first) left MPPI's delta with NO
   restoring pull at all - if the baseline ever settles away from tRoad=0
   (observed: ~2 m sustained offset on the circle track, cause unconfirmed,
   possibly a lane-offset setting or a leftover avoidance maneuver), nothing
   corrected it, and it compounded into a road departure once an obstacle's
   avoidance delta pushed the same direction as the existing drift. This
   keeps a small non-zero pull so a persistent baseline offset gets slowly
   corrected instead of accumulating indefinitely. */
constexpr double LaneCenterWeight = 1.0;
constexpr double TerminalLaneCenterWeight = 3.0;

/* DEBUG toggle: set to false to disable MPPI (and the fallback/recovery
   steering paths) entirely and run with IPGDriver's own baseline steering
   only, completely untouched - useful for seeing how the vehicle handles
   the track/route with no additional control layered on top at all.
   Compile-time, like BestFeasibleRolloutMode-style toggles elsewhere in
   this file: flip and rebuild. */
constexpr bool MppiEnabled = true;

double
Clamp(double value, double lower, double upper)
{
    return fmax(lower, fmin(upper, value));
}

double
WrapAngle(double angle)
{
    while (angle > Pi) {
        angle -= 2.0 * Pi;
    }
    while (angle < -Pi) {
        angle += 2.0 * Pi;
    }
    return angle;
}

/* FIX ("avoidance commits too late / fades out right when it's needed most"
   task): this used to be 3.0 s, deliberately set equal to
   MppiHorizon*MppiStep (also 3.0 s). That means the obstacle only entered
   AnyCollisionRiskSensed()'s trigger at the exact moment it was also just
   crossing INTO the edge of MPPI's own planning horizon - so at the instant
   avoidance first engages, the rollout can barely "see" the obstacle at
   all, let alone plan a gradual, early lateral shift around it. Observed on
   the expressway log: avoidance delta was already near max at first
   trigger (13.5 deg) but tRoad had barely moved yet, then the delta faded
   (8.5 deg, then 1.2 deg) as the car closed in - not because the danger was
   decreasing, but because by then the maneuver was mostly already "spent"
   inside a horizon that had no slack left over. Triggering 1+ second before
   the obstacle would even enter the horizon gives MPPI room to plan the
   whole maneuver as a smooth curve from the start, instead of committing
   hard right as the object appears and then running out of horizon to
   finish the job. */
constexpr double CollisionRiskTimeToCollision = 4.0;

/* Rollout visualization playback (see "animation... shows the car moving,
   and all the sampled paths" task). Every RolloutCaptureInterval seconds
   while MPPI is actually engaged, one frame (all 256 sampled paths + the
   vehicle's own position) is appended to the rollout CSV, instead of the
   earlier one-shot single snapshot. MaxRolloutFrames is a hard cap so a
   long maneuver can't grow the file without bound; capture just stops past
   that point (logged, not silent) rather than slowing the sim down.
   FIX ("increase the amount of samples ... double or triple feasible" /
   "can you make it double again" tasks): halved twice now, 0.5s -> 0.25s
   -> 0.125s (4x the original density) for a denser capture (the video-
   overlay camera interpolates between frames, but the actual path/road/
   obstacle content still only updates once per captured frame, so a
   shorter interval is what actually makes that content itself look more
   continuous). MaxRolloutFrames doubled alongside it each time so the
   total capture DURATION cap stays the same (120 s) rather than shrinking
   as resolution increases - CSV file size scales with frame count, so
   expect roughly 4x the original file size at this interval. */
constexpr double RolloutCaptureInterval = 0.125; /* [s] */
constexpr int MaxRolloutFrames = 960;            /* 120 s of capture at the interval above */

/* FIX ("MPPI kicks in on a 90-degree turn for an obstacle that isn't
   really in the way" task): generous buffer beyond the current lane/
   corridor's own edges (CurrentLeftRoadLimit/CurrentRightRoadLimit) used by
   the TTC gate below. Wide enough to still catch an obstacle sitting just
   off the lane edge (partially on the shoulder, sensor noise, etc.), but
   nowhere near wide enough to catch something that's actually on a
   different road segment entirely (e.g. around a sharp corner). */
constexpr double CollisionRiskLateralCorridorMargin = 2.0; /* [m] */

/* "Senses collision" gate (see doc/PROJECT_CONTEXT.md "only override on
   sensed collision" / "detection too late at high speed" tasks). Two
   checks, either one triggers:
   1. ANY currently-detected object's raw, un-projected sensor reading
      (dx, dy) falls inside the same normalized avoidance envelope
      ObstacleCost() itself treats as its "penetration zone"
      (normalizedDistanceSquared < 1.0) - a fixed geometric danger zone,
      correct at any speed. Left un-gated by corridor width deliberately:
      by the time an object is this close in the raw sensor frame, it's a
      real, immediate risk regardless of road geometry.
   2. Speed-aware: time-to-collision (dx / speed) is under
      CollisionRiskTimeToCollision. A FIXED distance alone gives less and
      less warning time as speed increases - at 25 m and 10 m/s that's
      2.5 s of warning, but at 40 m/s it's under 0.6 s ("swerving too
      late"). This makes the trigger distance grow with speed instead of
      staying fixed.
      FIX ("90-degree turn" task): this branch used to have NO lateral
      gate at all (tried the RAW `|dy| < obstacle.radius` gate first, and
      removed it - see doc/PROJECT_CONTEXT.md "circle path avoidance
      failure" task - because on a curving road obstacle.dy in the
      vehicle's own current-heading sensor frame stays large right up
      until the vehicle's heading swings to face the object, so that gate
      missed real curve-ahead threats). The fix here is not to remove the
      gate again, but to gate on the ROAD-FRAME position instead of the
      raw sensor-frame one: ObstacleAbsolutePosition() already rotates
      dx/dy by the current heading-relative-to-road, so an object dead
      ahead on a gently curving road (where heading tracks the curve
      fairly closely) still reads as "in the corridor" and triggers, while
      an object that's actually around a sharp 90-degree corner - whose
      raw dx looks close only because it's a straight-line distance
      through open space, not along the road - resolves to a road-frame
      lateral position tens of meters outside the current lane, and
      correctly does NOT trigger. */
bool
AnyCollisionRiskSensed(double speed)
{
    for (int i = 0; i < SensorObstacleCount; ++i) {
        const Obstacle &obstacle = SensorObstacles[i];
        if (!obstacle.valid) {
            continue;
        }
        const double longitudinalSeparation = obstacle.dx / ObstacleLongitudinalRadius;
        const double lateralSeparation = obstacle.dy / obstacle.radius;
        const double normalizedDistanceSquared =
            longitudinalSeparation * longitudinalSeparation
            + lateralSeparation * lateralSeparation;
        if (normalizedDistanceSquared < 1.0) {
            return true;
        }
        if (speed > 0.1 && obstacle.dx > 0.0
            && obstacle.dx / speed < CollisionRiskTimeToCollision) {
            double obstacleRoadDistance;
            double obstacleLateralPosition;
            ObstacleAbsolutePosition(obstacle, obstacleRoadDistance, obstacleLateralPosition);
            const double corridorLimit = (obstacleLateralPosition >= 0.0
                ? CurrentLeftRoadLimit : CurrentRightRoadLimit)
                + CollisionRiskLateralCorridorMargin;
            if (fabs(obstacleLateralPosition) < corridorLimit) {
                return true;
            }
        }
    }
    return false;
}

/* How close back to lane-center counts as "reconverged" - see
   NeedsMppiEngagement() below. */
constexpr double MppiReconvergenceLateralTolerance = 0.3; /* [m] */

/* Whether MPPI is currently inside an obstacle-avoidance-and-return-to-route
   episode - see NeedsMppiEngagement() below. Only ever set true by an
   actually sensed collision risk, never by lateral position alone. A
   generic lateral offset with no obstacle behind it (e.g. IPGDriver's own
   baseline running wide on a sharp turn after a long stall - observed to
   happen with MPPI correctly disengaged the whole time) is explicitly NOT
   MPPI's problem to fix; IPGDriver's own steering is never overridden for
   being merely off-center or close to the edge on its own (see
   doc/PROJECT_CONTEXT.md "only trigger on an actual obstacle, not
   proximity to the edge" task) - that's IPGDriver's job, full stop. The
   separate hard EdgeGuardrail override below is unaffected by this; it's
   a distinct last-resort safety net, not an MPPI engagement condition. */
bool RecoveringFromObstacle = false;

/* MPPI engages on a sensed collision risk (which also marks the start of a
   recovery episode), OR - only while still inside that episode - until the
   vehicle has reconverged to lane center. Once reconverged (or if no
   episode is active at all), MPPI stays off regardless of how large
   lateralPosition is; see the comment on RecoveringFromObstacle above for
   why that's deliberate rather than an oversight. This is a state
   transition, not a timer: unlike an earlier fixed grace-period attempt,
   there's no arbitrary duration to tune - the episode ends exactly when
   the car is actually back near center. Delta stacking on top of an
   already-large baseline (the original reason for restricting this) is
   handled separately below by MppiDeltaAuthorityScale(), not by narrowing
   when MPPI is allowed to run. */
bool
NeedsMppiEngagement(double speed, double lateralPosition, double dt)
{
    (void)dt;
    if (AnyCollisionRiskSensed(speed)) {
        RecoveringFromObstacle = true;
        return true;
    }
    if (RecoveringFromObstacle) {
        if (fabs(lateralPosition) > MppiReconvergenceLateralTolerance) {
            return true;
        }
        RecoveringFromObstacle = false;
    }
    return false;
}

/* See NeedsMppiEngagement() above - this is the actual fix for MPPI's delta
   stacking on top of an already-large baseline command. Below
   MppiDeltaAuthorityTaperStart, MPPI has its full +-SteeringLimit authority
   (normal obstacle-avoidance case, baseline near lane-center). Above
   MppiDeltaAuthorityTaperEnd, MPPI contributes nothing at all - baseline is
   already committing serious steering angle to execute a turn on its own,
   and adding more on top is how the vehicle ends up commanded well past
   what it can physically track. In between, authority tapers linearly
   rather than cutting off sharply, consistent with the smooth (not
   cliff-edge) cost shaping used everywhere else in this file. */
constexpr double MppiDeltaAuthorityTaperStart = 20.0 * Pi / 180.0; /* [rad] */
constexpr double MppiDeltaAuthorityTaperEnd = 40.0 * Pi / 180.0;   /* [rad] */

double
MppiDeltaAuthorityScale(double baselineSteering)
{
    const double magnitude = fabs(baselineSteering);
    if (magnitude <= MppiDeltaAuthorityTaperStart) {
        return 1.0;
    }
    if (magnitude >= MppiDeltaAuthorityTaperEnd) {
        return 0.0;
    }
    return (MppiDeltaAuthorityTaperEnd - magnitude)
        / (MppiDeltaAuthorityTaperEnd - MppiDeltaAuthorityTaperStart);
}

double
ObstacleCost(double roadDistance, double lateralPosition)
{
    double totalCost = 0.0;

    for (int i = 0; i < SensorObstacleCount; ++i) {
        const Obstacle &obstacle = SensorObstacles[i];
        if (!obstacle.valid) {
            continue;
        }

        double obstacleRoadDistance;
        double obstacleLateralPosition;
        ObstacleAbsolutePosition(obstacle, obstacleRoadDistance, obstacleLateralPosition);
        const double longitudinalSeparation =
            (roadDistance - obstacleRoadDistance) / ObstacleLongitudinalRadius;
        const double lateralSeparation =
            (lateralPosition - obstacleLateralPosition) / obstacle.radius;
        const double normalizedDistanceSquared =
            longitudinalSeparation * longitudinalSeparation
            + lateralSeparation * lateralSeparation;

        double cost = 250.0 * exp(-0.5 * normalizedDistanceSquared);
        /* FIX: passingGate previously only depended on longitudinal
           distance, so the "prefer to pass on the left" pull toward
           PreferredPassingLateralPosition activated for ANY object within
           ObstacleLongitudinalRadius, even ones tens of meters off to the
           side (e.g. dy=30 m) that pose no real collision risk - causing
           unwanted lane drift whenever such an object happened to be
           nearby in distance. lateralGate (reusing the same
           lateralSeparation already computed above, so no extra cost)
           makes the passing preference fade out for objects that aren't
           actually near the vehicle's path, the same envelope the
           collision cost itself already uses. */
        const double passingGate = exp(-0.5 * longitudinalSeparation * longitudinalSeparation);
        const double lateralGate = exp(-0.5 * lateralSeparation * lateralSeparation);
        const double passingError = lateralPosition - PreferredPassingLateralPosition;
        cost += 3.0 * passingGate * lateralGate * passingError * passingError;
        if (normalizedDistanceSquared < 1.0) {
            const double penetration = 1.0 - normalizedDistanceSquared;
            cost += 2000.0 + 8000.0 * penetration * penetration;
        }
        totalCost += cost;
    }

    return totalCost;
}

/* roadDistance is intentionally unused: CurrentLeftRoadLimit/
   CurrentRightRoadLimit are a per-cycle snapshot (see above), not a
   function of the predicted rollout position - kept as a parameter so the
   call sites (which iterate over predictedDistance/predictedLateralPosition
   pairs) don't need special-casing. */
double
RoadBoundaryCost(double /* roadDistance */, double lateralPosition)
{
    const bool isRight = lateralPosition < 0.0;
    const double hardLimit = isRight ? CurrentRightRoadLimit : CurrentLeftRoadLimit;
    const double softLimit = fmax(0.0, hardLimit - RoadSafetyMargin);
    const double softWeight = isRight ? RightRoadEdgeSoftPenaltyWeight : RoadEdgeSoftPenaltyWeight;
    const double hardWeight = isRight ? RightRoadEdgeHardPenaltyWeight : RoadEdgeHardPenaltyWeight;
    const double absoluteLateralPosition = fabs(lateralPosition);
    double cost = 0.0;
    if (absoluteLateralPosition > softLimit) {
        const double excess = absoluteLateralPosition - softLimit;
        cost += softWeight * excess * excess;
    }
    if (absoluteLateralPosition > hardLimit) {
        const double excess = absoluteLateralPosition - hardLimit;
        cost += hardWeight * (1.0 + excess * excess);
    }
    return cost;
}

/* Curvature-aware rollout: without this, the kinematic model below tracks
   predictedHeading relative to a single ROAD HEADING SNAPSHOT taken at the
   start of the horizon (t=0), so on a curving road it silently assumes the
   road keeps going straight in whatever direction it currently points -
   increasingly wrong the further the 4.5 s/~90 m horizon looks ahead. This
   reads the road's own curvature (1/m) at the predicted point so the
   rollout can bend predictedHeading's reference along with the actual
   road, not just a frozen initial direction.
   Sign convention: matches the existing t-positive-left assumption used
   throughout (see PreferredPassingLateralPosition); unverified empirically
   for curveXY specifically - if the vehicle drifts the wrong way on a
   curve, flip the sign where this is used below. Returns 0.0 (today's
   straight-road assumption) when curvature data isn't available. */
double
RoadCurvature(double roadDistance, double lateralPosition)
{
    if (MppiRoadEval == nullptr) {
        return 0.0;
    }

    tRoadRouteIn rIn{};
    tRoadRouteOut rOut{};
    rIn.st[0] = RoadEvaluationSOffset + roadDistance;
    rIn.st[1] = lateralPosition;

    if (RoadRouteEval(MppiRoadEval, nullptr, RIT_ST, &rIn, &rOut) != ROAD_Ok
        || !std::isfinite(rOut.curveXY)) {
        return 0.0;
    }

    return rOut.curveXY;
}

/* Converts a (roadDistance, lateralPosition) rollout point into global (x, y)
   - see SteeringMppi::AppendRolloutFrame() below ("doesn't look like the
   actual path" task). Everything MPPI reasons about internally is in
   road-frame (s, t): s is arc length along the route, t is lateral offset
   from lane center. That's the right frame for cost/control math, but
   plotting s/t directly only looks like the real path on a dead-straight
   road - on any curve, s/t is a "straightened out" view where the road
   itself is always a flat horizontal line, so a real curving maneuver
   looks nothing like what the car actually traces on the ground. Global
   (x, y) is what actually matches the driving screen. */
void
WorldPositionAt(double roadDistance, double lateralPosition, double &worldX, double &worldY)
{
    worldX = 0.0;
    worldY = 0.0;
    if (MppiRoadEval == nullptr) {
        return;
    }
    tRoadRouteIn rIn{};
    tRoadRouteOut rOut{};
    rIn.st[0] = RoadEvaluationSOffset + roadDistance;
    rIn.st[1] = lateralPosition;
    if (RoadRouteEval(MppiRoadEval, nullptr, RIT_ST, &rIn, &rOut) == ROAD_Ok
        && std::isfinite(rOut.xyz[0]) && std::isfinite(rOut.xyz[1])) {
        worldX = rOut.xyz[0];
        worldY = rOut.xyz[1];
    }
}

struct MppiResult {
    double Command;
    double BestCost;
    bool Valid;
};

class SteeringMppi {
public:
    SteeringMppi()
        : RandomGenerator(42u), NormalDistribution(0.0, MppiNoiseStd)
    {
        Reset();
    }

    void Reset()
    {
        NominalControl.fill(0.0);
        Costs.fill(0.0);
        RandomGenerator.seed(42u);
        NormalDistribution.reset();
    }

    /* baselineSteering: IPGDriver's own steering-wheel command for this
       cycle (see doc/PROJECT_CONTEXT.md "separate road-following from
       obstacle avoidance" task). MPPI no longer tries to track lane-center
       on its own (LaneCenterWeight below is 0) - that's IPGDriver's job,
       with its own steering authority, unrestricted by SteeringLimit.
       MPPI's sampled `steering` is now purely an avoidance DELTA, added to
       baselineSteering both in the rollout's kinematics (held constant
       across the horizon - a fair approximation on a constant-or-slowly-
       varying-radius curve) and in the executed command (see
       User_DrivMan_Calc). This is why a tight curve that exceeds
       SteeringLimit on its own can still be followed: the baseline
       supplies the large turn, MPPI only adds a small avoidance nudge. */
    MppiResult Update(double roadDistance, double lateralPosition,
                      double headingRelativeToRoad, double speed,
                      double previousAppliedSteering, double baselineSteering)
    {
        double minimumCost = std::numeric_limits<double>::infinity();
        const double correlationScale = sqrt(1.0 - MppiNoiseCorrelation * MppiNoiseCorrelation);
        const double noiseVariance = MppiNoiseStd * MppiNoiseStd;

        for (int sample = 0; sample < MppiSamples; ++sample) {
            double predictedDistance = roadDistance;
            double predictedLateralPosition = lateralPosition;
            double predictedHeading = headingRelativeToRoad;
            double previousControl = previousAppliedSteering;
            double previousNoise = 0.0;
            double cost = 0.0;

            for (int step = 0; step < MppiHorizon; ++step) {
                double noise = 0.0;
                if (sample != 0) {
                    const double whiteNoise = NormalDistribution(RandomGenerator);
                    noise = MppiNoiseCorrelation * previousNoise + correlationScale * whiteNoise;
                }

                const double steering = Clamp(NominalControl[step] + noise,
                                              -SteeringLimit, SteeringLimit);
                noise = steering - NominalControl[step];
                Noise[sample][step] = noise;
                previousNoise = noise;

                const double frontWheelAngle = (baselineSteering + steering) / MppiSteeringRatio;
                const double roadCurvature = RoadCurvature(predictedDistance, predictedLateralPosition);
                const double stepDistance = speed * cos(predictedHeading) * MppiStep;
                predictedDistance += stepDistance;
                predictedLateralPosition += speed * sin(predictedHeading) * MppiStep;
                predictedHeading = WrapAngle(predictedHeading
                    + speed / MppiWheelbase * tan(frontWheelAngle) * MppiStep
                    - roadCurvature * stepDistance);

                SampleRoadDistance[sample][step] = predictedDistance;
                SampleLateralPosition[sample][step] = predictedLateralPosition;

                const double lateralError = -predictedLateralPosition;
                const double headingError = WrapAngle(-predictedHeading);
                const double controlChange = steering - previousControl;

                const double runningCost =
                    LaneCenterWeight * lateralError * lateralError
                    + 60.0 * headingError * headingError
                    + 0.2 * steering * steering
                    + 0.5 * controlChange * controlChange
                    + ObstacleCost(predictedDistance, predictedLateralPosition)
                    + RoadBoundaryCost(predictedDistance, predictedLateralPosition);
                const double explorationCost = MppiTemperature
                    * NominalControl[step] * noise / noiseVariance;
                cost += MppiStep * (runningCost + explorationCost);
                previousControl = steering;
            }

            const double terminalLateralError = -predictedLateralPosition;
            const double terminalHeadingError = WrapAngle(-predictedHeading);
            cost += TerminalLaneCenterWeight * terminalLateralError * terminalLateralError
                + 100.0 * terminalHeadingError * terminalHeadingError
                + ObstacleCost(predictedDistance, predictedLateralPosition)
                + RoadBoundaryCost(predictedDistance, predictedLateralPosition);

            Costs[sample] = cost;
            minimumCost = fmin(minimumCost, cost);
        }

        double weightSum = 0.0;
        Weights.fill(0.0);
        for (int sample = 0; sample < MppiSamples; ++sample) {
            const double scaledCost = (Costs[sample] - minimumCost) / MppiTemperature;
            if (scaledCost < 60.0) {
                Weights[sample] = exp(-scaledCost);
                weightSum += Weights[sample];
            }
        }

        if (!std::isfinite(weightSum) || weightSum <= 1.0e-12) {
            return {0.0, minimumCost, false};
        }

        for (int step = 0; step < MppiHorizon; ++step) {
            double weightedNoise = 0.0;
            for (int sample = 0; sample < MppiSamples; ++sample) {
                weightedNoise += Weights[sample] * Noise[sample][step];
            }
            NominalControl[step] = Clamp(
                NominalControl[step] + weightedNoise / weightSum,
                -SteeringLimit, SteeringLimit);
        }

        const double command = NominalControl[0];
        for (int step = 0; step < MppiHorizon - 1; ++step) {
            NominalControl[step] = NominalControl[step + 1];
        }
        NominalControl[MppiHorizon - 1] = NominalControl[MppiHorizon - 2];

        return {command, minimumCost, std::isfinite(command)};
    }

    /* Rollout visualization playback (see "animation... shows the car
       moving, and all the sampled paths" task): appends one FRAME - every
       sample's full predicted path plus its total cost from the MOST
       RECENT Update() call, plus the vehicle's own current position/
       heading - to a growing CSV. Called at most every
       RolloutCaptureInterval seconds while MPPI is engaged (throttled from
       User_DrivMan_Calc), not every 20 Hz cycle - that would produce an
       unusable amount of data. frameIndex == 0 (re)creates the file so a
       new run doesn't inherit a previous run's frames; every later frame
       appends.
       Each point is written in BOTH frames: (s, t) is road-frame (arc
       length, lateral offset) - the frame MPPI's cost function actually
       reasons in - and (x, y) is global world coordinates via
       WorldPositionAt(), which is what actually matches the driving
       screen's shape on a curving road (see "doesn't look like the actual
       path" task: s/t alone renders any curve as a flat line). Also emits
       a road-shape reference (kind=R: center/left/right) spanning the same
       s-range the samples reached, so the viewer can draw the real road
       geometry behind the paths for context; a single kind=F row carrying
       the vehicle's own world position/heading for that frame (sample
       field repurposed to "car", cost field repurposed to sim time); and
       one kind=O row per currently-tracked obstacle (sample field
       repurposed to its object id, cost field repurposed to its radius) -
       all sharing one flat table instead of several files to keep in
       sync. */
    void
    AppendRolloutFrame(const char *path, int frameIndex, double simTime,
                       double originRoadDistance, double originLateralPosition,
                       double leftLimit, double rightLimit,
                       double vehicleWorldX, double vehicleWorldY, double vehicleYaw,
                       double vehicleHeadingRelativeToRoad) const
    {
        FILE *file = fopen(path, frameIndex == 0 ? "w" : "a");
        if (file == nullptr) {
            return;
        }
        if (frameIndex == 0) {
            fprintf(file,
                "# left_limit=%.6f right_limit=%.6f horizon=%d samples=%d "
                "capture_interval=%.3f\n",
                leftLimit, rightLimit, MppiHorizon, MppiSamples, RolloutCaptureInterval);
            /* roadYaw (see "make the ego vehicle turn with the steering
               too" task): heading relative to the ROAD tangent, not global
               world yaw - what the "yaw" column already carries. In
               road-relative (s,t) view, "forward" is always +s by
               construction, so world yaw isn't the right angle to rotate
               the ego marker by there; this is. Only meaningful on F
               (vehicle) rows - 0 elsewhere. */
            fprintf(file, "kind,sample,cost,step,s,t,x,y,frame,yaw,roadYaw\n");
        }

        fprintf(file, "F,car,%.6f,0,%.6f,%.6f,%.6f,%.6f,%d,%.6f,%.6f\n",
            simTime, originRoadDistance, originLateralPosition,
            vehicleWorldX, vehicleWorldY, frameIndex, vehicleYaw, vehicleHeadingRelativeToRoad);

        double minRoadDistance = originRoadDistance;
        double maxRoadDistance = originRoadDistance;
        for (int sample = 0; sample < MppiSamples; ++sample) {
            for (int step = 0; step < MppiHorizon; ++step) {
                minRoadDistance = fmin(minRoadDistance, SampleRoadDistance[sample][step]);
                maxRoadDistance = fmax(maxRoadDistance, SampleRoadDistance[sample][step]);
            }
        }

        /* Obstacle markers (see "add ... the obstacle where it is" task):
           cost field is repurposed to carry the obstacle's radius (for
           drawing it at roughly the right size), same one-flat-table
           reasoning as the F row above. */
        for (int i = 0; i < SensorObstacleCount; ++i) {
            const Obstacle &obstacle = SensorObstacles[i];
            if (!obstacle.valid) {
                continue;
            }
            double obstacleRoadDistance;
            double obstacleLateralPosition;
            ObstacleAbsolutePosition(obstacle, obstacleRoadDistance, obstacleLateralPosition);
            minRoadDistance = fmin(minRoadDistance, obstacleRoadDistance);
            maxRoadDistance = fmax(maxRoadDistance, obstacleRoadDistance);
            double obstacleWorldX;
            double obstacleWorldY;
            WorldPositionAt(obstacleRoadDistance, obstacleLateralPosition, obstacleWorldX, obstacleWorldY);
            fprintf(file, "O,%d,%.6f,0,%.6f,%.6f,%.6f,%.6f,%d,0,0\n",
                obstacle.objId, obstacle.radius, obstacleRoadDistance, obstacleLateralPosition,
                obstacleWorldX, obstacleWorldY, frameIndex);
        }

        for (int sample = 0; sample < MppiSamples; ++sample) {
            for (int step = 0; step < MppiHorizon; ++step) {
                const double s = SampleRoadDistance[sample][step];
                const double t = SampleLateralPosition[sample][step];
                double worldX;
                double worldY;
                WorldPositionAt(s, t, worldX, worldY);
                fprintf(file, "S,%d,%.6f,%d,%.6f,%.6f,%.6f,%.6f,%d,0,0\n",
                    sample, Costs[sample], step, s, t, worldX, worldY, frameIndex);
            }
        }

        constexpr int ReferencePoints = 60;
        const double referenceSpan = fmax(1.0, maxRoadDistance - minRoadDistance);
        const char *referenceKinds[3] = {"center", "left", "right"};
        const double referenceLateralPositions[3] = {0.0, leftLimit, -rightLimit};
        for (int i = 0; i < ReferencePoints; ++i) {
            const double s = minRoadDistance + referenceSpan * i / (ReferencePoints - 1);
            for (int side = 0; side < 3; ++side) {
                double worldX;
                double worldY;
                WorldPositionAt(s, referenceLateralPositions[side], worldX, worldY);
                fprintf(file, "R,%s,0,%d,%.6f,%.6f,%.6f,%.6f,%d,0,0\n",
                    referenceKinds[side], i, s, referenceLateralPositions[side],
                    worldX, worldY, frameIndex);
            }
        }

        fclose(file);
    }

private:
    std::array<double, MppiHorizon> NominalControl;
    std::array<std::array<double, MppiHorizon>, MppiSamples> Noise;
    std::array<std::array<double, MppiHorizon>, MppiSamples> SampleRoadDistance;
    std::array<std::array<double, MppiHorizon>, MppiSamples> SampleLateralPosition;
    std::array<double, MppiSamples> Costs;
    std::array<double, MppiSamples> Weights;
    std::mt19937 RandomGenerator;
    std::normal_distribution<double> NormalDistribution;
};

} /* namespace */

namespace {

constexpr char ObjectSensorName[] = "VehSensor_0";

int MountedObjectSensorIndex = -1;
tObjectSensor *MountedObjectSensor = nullptr;

void
InsertObstacleNearestFirst(std::array<Obstacle, MaxObstacles> &obstacles,
                           int &obstacleCount, Obstacle const &candidate)
{
    const double candidateDistanceSquared =
        candidate.dx * candidate.dx + candidate.dy * candidate.dy;
    int insertionIndex = 0;
    while (insertionIndex < obstacleCount) {
        const Obstacle &existing = obstacles[insertionIndex];
        const double existingDistanceSquared =
            existing.dx * existing.dx + existing.dy * existing.dy;
        if (candidateDistanceSquared < existingDistanceSquared) {
            break;
        }
        ++insertionIndex;
    }

    if (insertionIndex >= MaxObstacles) {
        return;
    }

    const int lastIndex = obstacleCount < MaxObstacles
        ? obstacleCount : MaxObstacles - 1;
    for (int i = lastIndex; i > insertionIndex; --i) {
        obstacles[i] = obstacles[i - 1];
    }
    obstacles[insertionIndex] = candidate;
    if (obstacleCount < MaxObstacles) {
        ++obstacleCount;
    }
}

void
ReadDetectedObstacles()
{
    std::array<Obstacle, MaxObstacles> detectedObstacles{};
    int detectedObstacleCount = 0;

    if (MountedObjectSensor != nullptr) {
        for (int i = 0; i < MountedObjectSensor->nObsvObjects; ++i) {
            const int objectId = MountedObjectSensor->ObsvObjects[i];
            tObjectSensorObj *object =
                ObjectSensor_GetObjectByObjId(MountedObjectSensorIndex, objectId);

            if (object == nullptr || object->dtct == 0) {
                continue;
            }

            const double dx = object->NearPnt.ds[0];
            const double dy = object->NearPnt.ds[1];
            if (!std::isfinite(dx) || !std::isfinite(dy) || dx <= 0.0) {
                continue;
            }

            const Obstacle candidate = {
                true, dx, dy, ObstacleLateralRadius, object->ObjId
            };
            InsertObstacleNearestFirst(
                detectedObstacles, detectedObstacleCount, candidate);
        }
    }

    SensorObstacles = detectedObstacles;
    SensorObstacleCount = detectedObstacleCount;
    User.Out[8] = NearestObstacleDx();
    User.Out[9] = NearestObstacleDy();
}

} /* namespace */


/*
 * User_Init_First ()
 *
 * First, low level initialization of the User module
 *
 * Call:
 * - one times at start of program
 * - no realtime conditions
 *
 */

int
User_Init_First(void)
{
    memset(&User, 0, sizeof(User));

    return 0;
}

/*
 * User_PrintUsage ()
 *
 * Print the user/application specific program arguments
 */

void
User_PrintUsage(char const *Pgm)
{
    /* REMARK: 1 log statement for each usage line, no line breaks */
    LogUsage("\n");
    LogUsage("Usage: %s [options] [testrun]\n", Pgm);
    LogUsage("Options:\n");

#if defined(CM_HIL)
    {
        tIOConfig const *cf;
        char const      *defio = IO_GetDefault();
        LogUsage(" -io %-12s Default I/O configuration (%s)\n", "default",
            (defio != NULL && strcmp(defio, "none") != 0) ? defio : "minimal I/O");
        for (cf = IO_GetConfigurations(); cf->Name != NULL; cf++) {
            LogUsage(" -io %-12s %s\n", cf->Name, cf->Description);
        }
    }
#endif
}

/*
 * User_ScanCmdLine ()
 *
 * Scan application specific command line arguments
 *
 * Return:
 * - argv: last unscanned argument
 * - NULL: error or unknown argument
 */

char **
User_ScanCmdLine(int argc, char **argv)
{
    char const *Pgm = argv[0];

    /* I/O configuration to be used in case no configuration was
       specified on the command line. */
    IO_SelectDefault("default" /* or "brakecu", "stwheel", "can,flexray" etc. */);

    while (*++argv) {
        if (strcmp(*argv, "-io") == 0 && argv[1] != NULL) {
            if (IO_Select(*++argv) != 0) {
                return NULL;
            }
        } else if (strcmp(*argv, "-h") == 0 || strcmp(*argv, "-help") == 0) {
            User_PrintUsage(Pgm);
            SimCore_PrintUsage(Pgm); /* Possible exit(), depending on CM-platform! */
            return NULL;
        } else if ((*argv)[0] == '-') {
            LogErrF(EC_General, "Unknown option '%s'", *argv);
            return NULL;
        } else {
            break;
        }
    }

    return argv;
}

/*
 * User_Init ()
 *
 * Basic initialization of the module User.o
 *
 * Call:
 * - once at program start
 * - no realtime conditions
 */

int
User_Init(void)
{
    printf("My code is running!\n");
    return 0;
}

int
User_Register(void)
{

    /* @@PLUGIN-BEGIN-REGISTER@@ - Automatically generated code - don't edit! */
    /* @@PLUGIN-END@@ */

    return 0;
}

/*
 * User_DeclQuants ()
 *
 * Add user specific quantities to the dictionary
 *
 * Call:
 * - once at program start
 * - no realtime conditions
 */

void
User_DeclQuants(void)
{
    int i;

    for (i = 0; i < N_USEROUTPUT; i++) {
        char sbuf[32];
        sprintf(sbuf, "UserOut_%02d", i);
        DDefDouble(NULL, sbuf, "", &User.Out[i], DVA_IO_Out);
    }
}

/*
 * User_Param_Add ()
 *
 * Update all modified application specific parameters in the test stand
 * parameter file (ECUParameters).
 *
 * If the variable SimCore.TestRig.ECUParam.Modified is set to 1 somewhere
 * else CarMaker calls this function to let the user add or change all
 * necessary entries before the file is written.
 * So, if writing the ECUParam file is necessary, set ECUParam.Modified to 1.
 * The next Test Run start or end, CarMaker calls this function and writes
 * the file to the harddisk.
 *
 * Call:
 * - in a separate thread (no realtime conditions)
 * - when starting a new Test Run
 */

int
User_Param_Add(void)
{
#if defined(CM_HIL)
    /* ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL) {
        return -1;
    }
#endif

    return 0;
}

/*
 * User_Param_Get ()
 *
 * Update all modified application specific parameters from the test stand
 * parameter file (ECUParameters).
 *
 * Call:
 * - in a separate thread (no realtime conditions)
 * - if User_Param_Get() wasn't called
 * - when starting a new Test Run, if
 *   - the files SimParameters and/or
 *   - ECUParameters
 *   are modified since last reading
 *
 * return values:
 *  0	ok
 * -1	no testrig parameter file
 * -2	testrig parameter error
 * -3	i/o configuration specific error
 * -4	no simulation parameters
 * -5	simulation parameters error
 * -6	Fail Safe Tester parameter/init error
 */

int
User_Param_Get(void)
{
    int rv = 0;

    if (RT_ACTIVE) {

        /*** testrig / ECU parameters */
        if (SimCore.TestRig.ECUParam.Inf == NULL) {
            return -1;
        }
        if (IO_Param_Get(SimCore.TestRig.ECUParam.Inf) != 0) {
            rv = -2;
        }
    }

    /*** simulation parameters */
    if (SimCore.TestRig.SimParam.Inf == NULL) {
        return -4;
    }

    return rv;
}

/*
 * User_TestRun_Start_atBegin ()
 *
 * Special things before a new simulation starts like
 * - reset user variables to their default values
 * - reset counters
 * - ...
 *
 * Call:
 * - in separate thread (no realtime conditions)
 * - when starting a new Test Run
 * - after (standard) Info Files are read in
 * - before reading parameters for Environment, DrivMan, Car, ...
 *   the models are NOT in the simulation-can-start-now state
 *   (after Start(), before StaticCond())
 */

int
User_TestRun_Start_atBegin(void)
{
    int rv = 0;
    int i;

    for (i = 0; i < N_USEROUTPUT; i++) {
        User.Out[i] = 0.0;
    }

    if (IO_None) {
        return rv;
    }
#if defined(CM_HIL)

    if (FST_New(SimCore.TestRig.ECUParam.Inf) != 0) {
        rv = -6;
    }
#endif

    return rv;
}


/*
 * User_TestRun_Start_atEnd ()
 *
 * Special things before a new simulation starts like
 * - reset user variables to their default values
 * - reset counters
 * - ...
 *
 * Call:
 * - in separate thread (no realtime conditions)
 * - when starting a new Test Run
 * - at the end, behind reading parameters for Environment, DrivMan,
 *   Car, ...
 *   the models are NOT in the simulation-can-start-now state
 *   (after Start(), before StaticCond())
 */

int
User_TestRun_Start_atEnd(void)
{
    if (MppiRoadEval != nullptr) {
        RoadDeleteRoadEval(MppiRoadEval);
        MppiRoadEval = nullptr;
    }

    MppiRoadEval = RoadNewRoadEval(
        Env.Road, ROAD_BUMP_NONE, ROAD_OT_WIDTH | ROAD_OT_LANES | ROAD_OT_CURVE_XY, "User.MPPI");
    if (MppiRoadEval == nullptr
        || RoadEvalSetRouteByObjId(MppiRoadEval, Env.Route.ObjId, 1) != ROAD_Ok) {
        LogWarnF(EC_Init,
            "MPPI road evaluation could not use the active route; using fallback width");
        if (MppiRoadEval != nullptr) {
            RoadDeleteRoadEval(MppiRoadEval);
            MppiRoadEval = nullptr;
        }
    }
    RoadEvaluationSOffset = 0.0;

    MountedObjectSensorIndex = ObjectSensor_FindIndexForName(ObjectSensorName);
    MountedObjectSensor = MountedObjectSensorIndex >= 0
        ? ObjectSensor_GetByIndex(MountedObjectSensorIndex) : nullptr;
    SensorObstacles = {};
    SensorObstacleCount = 0;
    SensorObstacleReferenceRoadDistance = 0.0;
    SensorObstacleReferenceLateralPosition = 0.0;
    SensorObstacleReferenceHeading = 0.0;
    CurrentRightRoadLimit = 0.5 * DefaultRoadWidth;
    CurrentLeftRoadLimit = 0.5 * DefaultRoadWidth;
    RecoveringFromObstacle = false;

    if (MountedObjectSensor == nullptr) {
        LogWarnF(EC_Init, "Object sensor '%s' was not found; continuing without sensor obstacle",
            ObjectSensorName);
        User.Out[8] = 999.0;
        User.Out[9] = 0.0;
    } else {
        Log("Object sensor '%s' initialized at index %d\n",
            ObjectSensorName, MountedObjectSensorIndex);
    }

    return 0;
}

/*
 * User_TestRun_Start_StaticCond_Calc ()
 *
 * called in non RT context
 */

int
User_TestRun_Start_StaticCond_Calc(void)
{
    return 0;
}

/*
 * User_TestRun_Start_Finalize ()
 *
 * called in RT context
 */

int
User_TestRun_Start_Finalize(void)
{
    return 0;
}

/*
 * User_TestRun_RampUp ()
 *
 * Perform a smooth transition of variables (e.g. I/O)
 * from their current state to the new Test Run.
 * This function is called repeatedly, once during each cycle, until
 * it returns true (or issues an error message), so the function should
 * return true if transitioning is done, false otherwise.
 *
 * In case of an error the function should issue an appropriate
 * error message and return false;
 *
 * Called in RT context, in state SCState_StartSim,
 * after preprocessing is done, before starting the engine.
 * Please note, that in this early initialization state no calculation
 * of the vehicle model takes place.
 */

int
User_TestRun_RampUp(double dt)
{
    int IsReady = 1;

    return IsReady;
}

/*
 * User_TestRun_End_First ()
 *
 * Invoked immediately after the end of a simulation is initiated,
 * but before data storage ends and before transitioning into SCState_Idle.
 * - Send Scratchpad-note
 * - ...
 *
 * Call:
 * - in main task, in the main loop (real-time conditions!)
 * - when a Test Run is finished (SimCore.State is SCState_End)
 */

int
User_TestRun_End_First(void)
{
    return 0;
}

/*
 * User_TestRun_End ()
 *
 * Special things after the end of a simulation like
 * - switch off an air compressor
 * - Write something to a file
 * - ...
 *
 * Call:
 * - in separate thread (no realtime conditions)
 * - when a test run is finished (SimCore.State is SCState_End<xyz>)
 */

int
User_TestRun_End(void)
{
    if (MppiRoadEval != nullptr) {
        RoadDeleteRoadEval(MppiRoadEval);
        MppiRoadEval = nullptr;
    }

    return 0;
}

/*
 * User_In ()
 *
 * Assign quantities of the i/o vector to model variables
 *
 * Call:
 * - in the main loop
 * - pay attention to realtime condition
 * - just after IO_In()
 */

void
User_In(unsigned const CycleNo)
{
    if (SimCore.State != SCState_Simulate) {
        return;
    }
}

/*
 * User_DrivMan_Calc ()
 *
 * called
 * - in RT context
 * - after DrivMan_Calc()
 */

int
User_DrivMan_Calc(double dt)
{
    // test to see if this function is called
    static bool firstDrivMan = true;
    if (firstDrivMan) {
        Log("User_DrivMan_Calc() is running!\n");
        Log(MppiEnabled
            ? "MPPI steering: ENABLED\n"
            : "MPPI steering: DISABLED - running IPGDriver baseline steering untouched\n");
        firstDrivMan = false;
    }

    if (!MppiEnabled) {
        /* Leave DrivMan.Steering exactly as the core DrivMan_Calc() (which
           already ran this cycle) computed it - no MPPI, no fallback, no
           recovery, no obstacle avoidance. Pure baseline route-following. */
        return 0;
    }

    static SteeringMppi mppiController;
    static double controlStartRoadPosition = 0.0;
    static bool controlReferenceInitialized = false;
    static double steeringCommand = 0.0;
    static double previousSteeringVelocity = 0.0;
    static double mppiUpdateTimer = 0.0;
    static double mppiRequestedSteering = 0.0;
    static double mppiBestCost = 0.0;
    static bool mppiValid = false;
    static bool hasLastValidMppiCommand = false;
    static double lastValidMppiCommand = 0.0;
    /* Tracks MPPI's own last DELTA (not the blended total) so the next
       Update() call's controlChange cost measures how much the AVOIDANCE
       contribution is changing, not the baseline's - see
       doc/PROJECT_CONTEXT.md "separate road-following from obstacle
       avoidance" task. */
    static double previousMppiDelta = 0.0;
    /* FIX (see doc/PROJECT_CONTEXT.md "steering runaway while stalled"
       task): ipgBaselineSteering is read from DrivMan.Steering.Ang at the
       TOP of this function, but WE write DrivMan.Steering.Ang =
       steeringCommand at the BOTTOM of the previous cycle - so it is not
       an independent signal once RECOVERY mode has run even once, it's
       partly our own prior output. RECOVERY previously did
       `ipgBaselineSteering + Clamp(lastValidMppiCommand, +-RecoverySteeringLimit)`
       EVERY cycle, using a freshly re-read (self-referential) baseline
       each time; since lastValidMppiCommand stays fixed and nonzero while
       stalled, this is a literal discrete integrator with no ceiling -
       observed climbing past 600 degrees over a ~30s stall. Snapshotting
       the baseline ONCE when a stall begins and holding that fixed value
       breaks the feedback loop entirely. */
    static bool stalledBaselineCaptured = false;
    static double stalledBaselineSteering = 0.0;
    static unsigned int logCounter = 0;
    /* Rollout visualization playback - see SteeringMppi::AppendRolloutFrame()
       and the "animation... shows the car moving" task. Frame 0 captures
       immediately on first engagement; later frames are throttled to
       RolloutCaptureInterval by rolloutCaptureTimer, and capture stops
       once rolloutFrameIndex reaches MaxRolloutFrames. */
    static double rolloutCaptureTimer = 0.0;
    static int rolloutFrameIndex = 0;
    static bool rolloutCapWarned = false;

    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving) {
        controlStartRoadPosition = 0.0;
        controlReferenceInitialized = false;
        steeringCommand = 0.0;
        previousSteeringVelocity = 0.0;
        mppiUpdateTimer = 0.0;
        mppiRequestedSteering = 0.0;
        mppiBestCost = 0.0;
        mppiValid = false;
        hasLastValidMppiCommand = false;
        lastValidMppiCommand = 0.0;
        previousMppiDelta = 0.0;
        stalledBaselineCaptured = false;
        stalledBaselineSteering = 0.0;
        mppiController.Reset();
        logCounter = 0;
        rolloutCaptureTimer = 0.0;
        rolloutFrameIndex = 0;
        rolloutCapWarned = false;
        User.Out[0] = 0.0;
        User.Out[1] = Vehicle.Road.Path.tRoad;
        User.Out[2] = -Vehicle.Road.Path.tRoad;
        User.Out[3] = 0.0;
        User.Out[4] = 0.0;
        User.Out[5] = 0.0;
        User.Out[6] = 0.0;
        User.Out[7] = 0.0;
        User.Out[8] = NearestObstacleDx();
        User.Out[9] = NearestObstacleDy();
        return 0;
    }

    /* IPGDriver's own road-following steering for this cycle, computed by
       the core DrivMan_Calc() that already ran before we get here - this
       is what carries the vehicle through a curve; MPPI below only adds a
       small avoidance delta on top (see doc/PROJECT_CONTEXT.md "separate
       road-following from obstacle avoidance" task). Captured before
       anything below overwrites DrivMan.Steering. */
    const double ipgBaselineSteering = DrivMan.Steering.Ang;

    if (dt <= 0.0 || Vehicle.Road.offRoute) {
        return 0;
    }

    const double vehicleSpeed = fabs(Vehicle.v);
    if (!controlReferenceInitialized && vehicleSpeed > 1.0) {
        controlStartRoadPosition = Vehicle.sRoad;
        controlReferenceInitialized = true;
        mppiUpdateTimer = MppiUpdatePeriod;
        mppiController.Reset();
    }

    const SpeedState speedState = !controlReferenceInitialized
        ? SpeedState::Startup
        : (vehicleSpeed < 1.0 ? SpeedState::Stalled : SpeedState::Moving);
    ControllerMode controllerMode = ControllerMode::Fallback;
    /* Whether anything below is actually adding a correction on top of
       (or in place of) IPGDriver's own steering this cycle - see the
       steeringCommand computation at the bottom of this function. Only
       true when MPPI has a valid delta, a real recovery correction is
       being added, or the guardrail fires. False (the common case, no
       obstacle nearby) means IPGDriver's own baseline is trusted
       completely, with no rate/acceleration smoothing from us either -
       see the "our own rate limiter was throttling raw IPGDriver output"
       task in doc/PROJECT_CONTEXT.md. */
    bool mppiIsActivelyCorrecting = false;

    const double distanceFromControlStart = controlReferenceInitialized
        ? Vehicle.sRoad - controlStartRoadPosition : 0.0;
    RoadEvaluationSOffset = controlReferenceInitialized
        ? controlStartRoadPosition : Vehicle.sRoad;
    SensorObstacleReferenceRoadDistance = distanceFromControlStart;
    SensorObstacleReferenceLateralPosition = Vehicle.Road.Path.tRoad;
    constexpr double desiredLateralPosition = 0.0;
    constexpr double desiredHeadingRelativeToRoad = 0.0;
    const double roadHeading = atan2(Vehicle.Road.Path.X_0[1], Vehicle.Road.Path.X_0[0]);
    const double headingRelativeToRoad = WrapAngle(Vehicle.Yaw - roadHeading);
    SensorObstacleReferenceHeading = headingRelativeToRoad;
    const double headingError = WrapAngle(
        desiredHeadingRelativeToRoad - headingRelativeToRoad);

    const double lateralError = desiredLateralPosition - Vehicle.Road.Path.tRoad;

    /* Snapshot this cycle's lane-relative road limits - see
       CurrentRightRoadLimit/CurrentLeftRoadLimit above for why this reads
       Vehicle.Road.Act/OnLeft (Path-relative, matching tRoad's own
       frame) instead of RoadRouteEval (Route-relative). */
    {
        const double actWidth = std::isfinite(Vehicle.Road.Act.Width)
                && Vehicle.Road.Act.Width > 0.0
            ? Vehicle.Road.Act.Width : DefaultRoadWidth;
        if (actWidth >= MinimumPlausibleActWidth) {
            const double leftLaneWidth = std::isfinite(Vehicle.Road.OnLeft.Width)
                    && Vehicle.Road.OnLeft.Width > 0.0
                ? Vehicle.Road.OnLeft.Width : 0.0;
            CurrentRightRoadLimit = 0.5 * actWidth;
            CurrentLeftRoadLimit = 0.5 * actWidth + leftLaneWidth;
        }
        /* else: implausibly narrow reading (see MinimumPlausibleActWidth
           above) - hold last cycle's CurrentRightRoadLimit/
           CurrentLeftRoadLimit rather than collapsing the guardrail and
           RoadBoundaryCost limits to a junction-node placeholder width. */
    }

    if (speedState == SpeedState::Moving) {
        /* Only let MPPI touch the steering at all if it actually senses a
           collision risk OR the car hasn't reconverged to lane-center yet
           (see doc/PROJECT_CONTEXT.md "only override on sensed collision" /
           "can't return to a curving route" tasks) - otherwise IPGDriver's
           own baseline passes straight through, untouched, and MPPI
           doesn't even run (saves the 256x60 rollout too). */
        if (NeedsMppiEngagement(vehicleSpeed, Vehicle.Road.Path.tRoad, dt)) {
            mppiUpdateTimer += dt;
            rolloutCaptureTimer += dt;
            if (mppiUpdateTimer >= MppiUpdatePeriod) {
                const MppiResult result = mppiController.Update(
                    distanceFromControlStart, Vehicle.Road.Path.tRoad,
                    headingRelativeToRoad, vehicleSpeed, previousMppiDelta, ipgBaselineSteering);
                mppiUpdateTimer = fmod(mppiUpdateTimer, MppiUpdatePeriod);
                mppiBestCost = result.BestCost;
                mppiValid = result.Valid;
                /* result.Command is MPPI's avoidance DELTA only; the executed
                   command adds it to IPGDriver's own road-following baseline
                   (see doc/PROJECT_CONTEXT.md "separate road-following from
                   obstacle avoidance" task). If MPPI itself is invalid, fall
                   back to the baseline alone rather than a standalone
                   absolute correction - see the RECOVERY/FALLBACK fix below
                   for why (dropping the baseline entirely caused the vehicle
                   to leave a curving road). The delta is scaled by
                   MppiDeltaAuthorityScale() (see NeedsMppiEngagement above)
                   so it can't stack unchecked on top of a baseline that's
                   already committing a large angle to execute a turn on its
                   own; the scaled value (not the raw one) is what gets
                   remembered for warm-starting and reconvergence bookkeeping
                   since it's what was actually applied. */
                const double scaledDelta = result.Command
                    * MppiDeltaAuthorityScale(ipgBaselineSteering);
                mppiRequestedSteering = result.Valid
                    ? ipgBaselineSteering + scaledDelta : ipgBaselineSteering;
                if (result.Valid) {
                    hasLastValidMppiCommand = true;
                    lastValidMppiCommand = scaledDelta;
                    previousMppiDelta = scaledDelta;
                }
                if (result.Valid && rolloutFrameIndex < MaxRolloutFrames
                    && (rolloutFrameIndex == 0 || rolloutCaptureTimer >= RolloutCaptureInterval)) {
                    mppiController.AppendRolloutFrame(
                        "Data/TestRun/.tmp_mppi_rollout.csv", rolloutFrameIndex, SimCore.Time,
                        distanceFromControlStart, Vehicle.Road.Path.tRoad,
                        CurrentLeftRoadLimit, CurrentRightRoadLimit,
                        Vehicle.Fr1A.t_0[0], Vehicle.Fr1A.t_0[1], Vehicle.Yaw,
                        headingRelativeToRoad);
                    rolloutCaptureTimer = 0.0;
                    if (rolloutFrameIndex == 0) {
                        Log("MPPI rollout capture started: "
                            "Data/TestRun/.tmp_mppi_rollout.csv\n");
                    }
                    ++rolloutFrameIndex;
                } else if (rolloutFrameIndex >= MaxRolloutFrames && !rolloutCapWarned) {
                    Log("MPPI rollout capture stopped at %d frames "
                        "(MaxRolloutFrames reached)\n", MaxRolloutFrames);
                    rolloutCapWarned = true;
                }
            }
            controllerMode = mppiValid ? ControllerMode::Mppi : ControllerMode::Fallback;
            mppiIsActivelyCorrecting = mppiValid;
        } else {
            mppiUpdateTimer = 0.0;
            mppiValid = false;
            mppiBestCost = 0.0;
            previousMppiDelta = 0.0;
            mppiRequestedSteering = ipgBaselineSteering;
            controllerMode = ControllerMode::Passthrough;
            mppiIsActivelyCorrecting = false;
        }
        /* Not stalled right now: the next stall (if any) should snapshot a
           fresh baseline, not reuse a stale one from a previous episode
           (see doc/PROJECT_CONTEXT.md "steering runaway while stalled"
           task). */
        stalledBaselineCaptured = false;
    } else if (speedState == SpeedState::Stalled) {
        /* FIX (see doc/PROJECT_CONTEXT.md "recovery went off a curving
           road" / "steering runaway while stalled" tasks): this used to
           add the recovery correction to a freshly re-read
           ipgBaselineSteering EVERY cycle - but ipgBaselineSteering is
           partly our own prior output (we write DrivMan.Steering.Ang each
           cycle, then read it back next cycle), so with a fixed nonzero
           lastValidMppiCommand this was a literal unbounded integrator
           (observed: climbed past 600 degrees over a ~30s stall). Snapshot
           the baseline ONCE when the stall begins and hold that fixed
           value for the correction to sit on top of, instead of
           re-reading a value that includes everything we've already added
           on previous stalled cycles.

           That freeze is only actually needed while a nonzero correction is
           being added on top every cycle - that's the specific thing that
           integrates. A stall with nothing to recover from (no residual
           obstacle-avoidance memory, e.g. waiting at a traffic light well
           after the reconvergence grace period has lapsed) doesn't need it:
           passing the live baseline straight through adds nothing per
           cycle, so it can't run away either. Freezing it anyway in that
           case was denying IPGDriver's own steering law any chance to keep
           adjusting while stopped (observed: a ~30s light stop right before
           a sharp turn held the frozen pre-stop angle the whole time, then
           had to snap ~8 degrees larger all at once the instant motion
           resumed, which the vehicle couldn't track without running wide -
           see doc/PROJECT_CONTEXT.md "baseline diverges from default driver
           after a long stall" task). */
        mppiValid = false;
        const double recoverySource = hasLastValidMppiCommand ? lastValidMppiCommand : 0.0;
        const double recoveryCorrection = Clamp(
            recoverySource, -RecoverySteeringLimit, RecoverySteeringLimit);
        if (fabs(recoveryCorrection) > 1.0e-9) {
            if (!stalledBaselineCaptured) {
                stalledBaselineSteering = ipgBaselineSteering;
                stalledBaselineCaptured = true;
            }
            mppiRequestedSteering = stalledBaselineSteering + recoveryCorrection;
            mppiIsActivelyCorrecting = true;
        } else {
            stalledBaselineCaptured = false;
            mppiRequestedSteering = ipgBaselineSteering;
            mppiIsActivelyCorrecting = false;
        }
        controllerMode = ControllerMode::Recovery;
    } else {
        /* STARTUP: no MPPI delta computed yet - baseline alone. */
        mppiValid = false;
        mppiRequestedSteering = ipgBaselineSteering;
        controllerMode = ControllerMode::Fallback;
    }

    /* Hard edge guardrail - see RoadEdgeGuardrailMargin above. Deliberately
       overrides mppiRequestedSteering itself (not steeringCommand
       directly), so the existing rate/acceleration limiting below still
       applies to it; this is a change of TARGET, not a bypass of the
       actuator smoothing. Positive tRoad is left (see
       PreferredPassingLateralPosition), and positive steering turns left,
       so drifting past the left edge must steer right (negative) and vice
       versa.

       FIX (explicit instruction: MPPI/any override should only ever engage
       for an actual on-road obstacle and the return-to-route afterward,
       never merely for being close to the edge on its own). This used to
       fire unconditionally, purely off tRoad, regardless of cause - the
       exact "too close to edge" trigger this project has repeatedly been
       told not to use. Gated on RecoveringFromObstacle (see
       NeedsMppiEngagement above) so a road departure with no obstacle
       behind it is left entirely to IPGDriver, same as everywhere else in
       this file now. */
    if (RecoveringFromObstacle) {
        const double leftGuardrailLimit = CurrentLeftRoadLimit - RoadEdgeGuardrailMargin;
        const double rightGuardrailLimit = CurrentRightRoadLimit - RoadEdgeGuardrailMargin;
        if (Vehicle.Road.Path.tRoad > leftGuardrailLimit) {
            const double excess = Vehicle.Road.Path.tRoad - leftGuardrailLimit;
            const double rampFraction = Clamp(excess / EdgeGuardrailRampDistance, 0.0, 1.0);
            mppiRequestedSteering = -EdgeGuardrailSteeringLimit * rampFraction;
            controllerMode = ControllerMode::EdgeGuardrail;
            mppiIsActivelyCorrecting = true;
        } else if (Vehicle.Road.Path.tRoad < -rightGuardrailLimit) {
            const double excess = -rightGuardrailLimit - Vehicle.Road.Path.tRoad;
            const double rampFraction = Clamp(excess / EdgeGuardrailRampDistance, 0.0, 1.0);
            mppiRequestedSteering = EdgeGuardrailSteeringLimit * rampFraction;
            controllerMode = ControllerMode::EdgeGuardrail;
            mppiIsActivelyCorrecting = true;
        }
    }

    const double previousSteeringCommand = steeringCommand;
    if (mppiIsActivelyCorrecting) {
        /* Only rate/acceleration-limit our OWN correction (MPPI's sampled
           delta, a recovery nudge, or the guardrail) - that's the signal
           that actually needs smoothing, since it can otherwise jump
           discontinuously between updates. */
        const double maximumSteeringStep = SteeringRateLimit * dt;
        const double steeringError = mppiRequestedSteering - steeringCommand;
        steeringCommand += Clamp(steeringError, -maximumSteeringStep, maximumSteeringStep);
    } else {
        /* FIX (explicit instruction, confirmed by testing: IPGDriver drives
           this scenario correctly when our controller is fully bypassed,
           but not when merely contributing zero delta through this
           function). IPGDriver has its own internal steering-actuator
           dynamics (Driver.Lat.StWhlAngleVelMax/AccMax in the TestRun,
           far faster than SteeringRateLimit above) - applying OUR rate
           limiter on top of that, even while adding nothing ourselves, is
           a second, much slower actuator model IPGDriver never asked for.
           At any single instant the two looked close in logged snapshots,
           but the resulting PHASE LAG compounds into real trajectory error
           over a fast maneuver (e.g. resuming a sharp turn from a stall).
           With no obstacle involved, mirror ipgBaselineSteering exactly -
           zero interference, matching MppiEnabled=false behavior. */
        steeringCommand = ipgBaselineSteering;
    }
    steeringCommand = Clamp(steeringCommand,
        -MaxAbsoluteSteeringCommand, MaxAbsoluteSteeringCommand);

    const double steeringVelocity = (steeringCommand - previousSteeringCommand) / dt;
    double steeringAcceleration = (steeringVelocity - previousSteeringVelocity) / dt;
    steeringAcceleration = fmax(-SteeringAccelerationLimit,
                                fmin(SteeringAccelerationLimit, steeringAcceleration));
    previousSteeringVelocity = steeringVelocity;

    DrivMan.Steering.SteerBy = DMSteerBy_Angle;
    DrivMan.Steering.Ang = steeringCommand;
    DrivMan.Steering.AngVel = steeringVelocity;
    DrivMan.Steering.AngAcc = steeringAcceleration;

    User.Out[0] = desiredLateralPosition;
    User.Out[1] = Vehicle.Road.Path.tRoad;
    User.Out[2] = lateralError;
    User.Out[3] = headingError;
    User.Out[4] = steeringCommand;
    User.Out[5] = mppiRequestedSteering;
    User.Out[6] = mppiBestCost;
    User.Out[7] = controllerMode == ControllerMode::Mppi ? 1.0
        : (controllerMode == ControllerMode::Recovery ? 2.0
        : (controllerMode == ControllerMode::Passthrough ? 3.0
        : (controllerMode == ControllerMode::EdgeGuardrail ? 4.0 : 0.0)));
    User.Out[8] = NearestObstacleDx();
    User.Out[9] = NearestObstacleDy();

    if (logCounter++ % 1000 == 0) {
        /* Reports EVERY tracked obstacle's resolved absolute road-frame
           lateral position (id@lateralPosition), not just the nearest
           one's raw sensor dx/dy - needed to answer "why did it swerve
           toward that side" from the log directly (e.g. confirming whether
           a second car is actually occupying the other side's gap) instead
           of inferring it blind. */
        char obstacleIds[160] = "none";
        if (SensorObstacleCount > 0) {
            size_t offset = 0;
            obstacleIds[0] = '\0';
            for (int i = 0; i < SensorObstacleCount; ++i) {
                double obstacleRoadPosition;
                double obstacleLateralPosition;
                ObstacleAbsolutePosition(
                    SensorObstacles[i], obstacleRoadPosition, obstacleLateralPosition);
                const int written = snprintf(
                    obstacleIds + offset, sizeof(obstacleIds) - offset,
                    i == 0 ? "%d@%.2f" : ",%d@%.2f",
                    SensorObstacles[i].objId, obstacleLateralPosition);
                if (written < 0
                    || static_cast<size_t>(written) >= sizeof(obstacleIds) - offset) {
                    break;
                }
                offset += static_cast<size_t>(written);
            }
        }

        const int nearestObjectId = SensorObstacleCount > 0
            ? SensorObstacles[0].objId : -1;
        const double currentRoadCurvature =
            RoadCurvature(distanceFromControlStart, Vehicle.Road.Path.tRoad);
        Log("Steering control: speedState=%s obstacleCount=%d nearest dx=%.1f m "
            "dy=%.3f m id=%d ids=%s command=%.2f deg cost=%.2f mode=%s "
            "tRoad=%.3f m curvature=%.5f 1/m baseline=%.2f deg delta=%.2f deg "
            "laneWidth=%.2f m leftLimit=%.2f m rightLimit=%.2f m\n",
            SpeedStateName(speedState), SensorObstacleCount,
            NearestObstacleDx(), NearestObstacleDy(), nearestObjectId, obstacleIds,
            steeringCommand * 180.0 / Pi, mppiBestCost,
            ControllerModeName(controllerMode),
            Vehicle.Road.Path.tRoad, currentRoadCurvature,
            ipgBaselineSteering * 180.0 / Pi, previousMppiDelta * 180.0 / Pi,
            Vehicle.Road.Act.Width, CurrentLeftRoadLimit, CurrentRightRoadLimit);
    }

    return 0;
}

/*
 * User_VehicleControl_Calc ()
 *
 * called
 * - in RT context
 * - after VehicleControl_Calc()
 */

int
User_VehicleControl_Calc(double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving) {
        return 0;
    }

    static bool first = true;

    if (first) {
        Log("BUILD STAMP: ZERO_INTERFERENCE_PASSTHROUGH_V41 (built %s %s)\n", __DATE__, __TIME__);
        Log("User_VehicleControl_Calc() is running!");
        first = false;
    }

    return 0;
}

/*
 * User_Brake_Calc ()
 *
 * called
 * - in RT context
 * - after Brake_Calc() in Vhcl_Calc()
 */

int
User_Brake_Calc(double dt)
{
    /* Modify the total brake torque from the brake system model Brake.Trq_tot[]
       or the target drive source torque from the brake control unit
       Brake.HydBrakeCU_IF.Trq_DriveSrc_trg[]
    */

    return 0;
}

/*
 * User_Traffic_Calc ()
 *
 * called
 * - in RT context
 * - after Traffic_Calc()
 */

int
User_Traffic_Calc(double dt)
{
    if (SimCore.State != SCState_Simulate) {
        return 0;
    }

    return 0;
}

/*
 * User_Calc ()
 *
 * called in RT context
 */

int
User_Calc(double dt)
{
    /* Starting with CM 6.0 User_Calc() will be invoked in EVERY simulation
       state. Uncomment the following line in order to restore the behaviour
       of CM 5.1 and earlier. */
    /*if (!UserCalcCalledByAppTestRunCalc) return 0;*/

    static int count = 0;

    if (count++ % 1000 == 0) {
        Log("Vehicle speed: %f\n", Vehicle.v);
    }

    ReadDetectedObstacles();

    return 0;
}

/*
 * User_Check_IsIdle ()
 *
 * Checking, if the simulation model is in idle conditions (stand still,
 * steering wheel angle zero, clutch pedal pressed, ...).
 * If reached idle state, the calculation of vehicle model and driving
 * maneuvers is stopped.
 * Ready for start new simulation.
 *
 * Return:
 * 1  idle state reached
 * 0  else
 *
 * Call:
 * - in main task, in the main loop
 * - pay attention to realtime condition
 * - while SimCore.State==SCState_EndIdleGet
 */

int
User_Check_IsIdle(int IsIdle)
{
    double val;

    /*** ECU / car model signals */

    /* vehicle and wheels: stand still */
    val = 0.5 * kmh2ms;
    if (Vehicle.v > val || fabs(Vehicle.Wheel[0]->vBelt) > val || fabs(Vehicle.Wheel[1]->vBelt) > val
        || fabs(Vehicle.Wheel[2]->vBelt) > val || fabs(Vehicle.Wheel[3]->vBelt) > val) {
        IsIdle = 0;
    }

    /* SteerAngle: drive straight forward position */
    val = 1.0 * deg2rad;
    if (Vehicle.Steering.Ang > val || Vehicle.Steering.Ang < -val) {
        IsIdle = 0;
    }

    return IsIdle;
}

/*
 * User_Out ()
 *
 * Assigns model quantities to variables of the i/o vector
 *
 * call:
 * - in the main loop
 * - pay attention to realtime condition
 * - just before IO_Out();
 */

void
User_Out(unsigned const CycleNo)
{

    if (SimCore.State != SCState_Simulate) {
        return;
    }
}

/*
 * User_ApoMsg_Eval ()
 *
 * Communication between the application and connected GUIs.
 * Evaluate messages from GUIs
 *
 * Call:
 * - in the main loop
 * - pay attention to realtime condition
 * - near the end of the main loop, if the function SimCore_ApoMsg_Eval()
 *    skips the message
 *
 * Return:
 *   0 : message evaluated
 *  -1 : message not handled
 */

int
User_ApoMsg_Eval(int Ch, char *Msg, int len, int who)
{
    if (Ch == ApoCh_CarMaker) {
#if defined(CM_HIL)
        /*** Fail Safe Tester */
        if (FST_ApoMsgEval(Ch, Msg, len) <= 0) {
            return 0;
        }
#endif
    }

    return -1;
}

/*
 * User_ApoMsg_Send ()
 *
 * Communication between the application and connected GUIs.
 * Sends messages to GUIs
 *
 * Call:
 * - near the end of the main loop, in MainThread_FinishCycle()
 * - pay attention to realtime condition
 */

void
User_ApoMsg_Send(double T, unsigned const CycleNo)
{
}

/*
 * User_ShutDown ()
 *
 * Prepare application for shut down
 *
 * Call:
 * - at end of program
 * - no realtime conditions
 */

int
User_ShutDown(int ShutDownForced)
{
    int IsDown = 0;

    /* Prepare application for shutdown and return that
       shutdown conditions are reached */
    if (1) {
        IsDown = 1;
    }

    return IsDown;
}

/*
 * User_End ()
 *
 * End all models of the user module
 *
 * Call:
 * - one times at end of program
 * - no realtime conditions
 */

int
User_End(void)
{
    return 0;
}

/*
 * User_Cleanup ()
 *
 * Cleanup function of the User module
 *
 * Call:
 * - one times at end of program, just before exit
 * - no realtime conditions
 */

void
User_Cleanup(void)
{
}
