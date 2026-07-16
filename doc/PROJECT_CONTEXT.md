# PROJECT_CONTEXT.md

# Instructions for AI assistants

Read this file completely before making any code changes.

Treat this document as the authoritative project design.

Do not re-implement features that are marked as completed.

When adding new functionality:
- preserve existing behavior
- update this document if architecture changes
- prefer incremental modifications over rewrites

# CarMaker MPPI Project

Author: Edison Suzuki

Last Updated:
2026-07-16

---

# 1. Project Goal

Develop a steering-only Model Predictive Path Integral (MPPI) controller in CarMaker Office using C++.

Final objective:

- Detect obstacles using CarMaker sensors.
- Avoid them using MPPI.
- Return smoothly to lane center.
- Compare against a simple proportional steering controller.

This project emphasizes understanding MPPI rather than simply obtaining obstacle avoidance.

---

# 2. Development Roadmap

Completed

✓ CarMaker project setup

✓ User.cpp integration

✓ Visual Studio build

✓ Logging

✓ User.Out plotting

✓ Simple proportional steering controller

✓ Sinusoidal reference tracking

✓ Steering-only MPPI

✓ Hard-coded obstacle avoidance

Current

→ Replace hard-coded obstacle with Object Sensor

Future

□ Multiple obstacles

□ Dynamic obstacles

□ Throttle + steering MPPI

□ Dynamic bicycle model

□ Controller comparison

---

# 3. Project Architecture

Controller entry point

User_DrivMan_Calc()

Current status

ACTIVE

Debug hook

User_VehicleControl_Calc()

Only used for debug messages.

Not part of steering control.

---

# 4. Controller History

Original controller

Simple proportional controller

steering =
    K_lat * lateralError
  + K_heading * headingError

Current controller

Steering-only MPPI

Fallback controller still exists and is used when MPPI returns invalid.

Low-speed recovery distinguishes startup from a stall after the vehicle has moved.

During a stall, steering uses a gentle command derived from the last valid MPPI command instead of saturated fallback steering.

---

# 5. MPPI Architecture

Current controller

SteeringMppi

Prediction model

Kinematic bicycle model

Prediction states

- road distance
- lateral position
- heading

Prediction input

Steering wheel angle

Optimization

- 256 sampled trajectories
- 2 second horizon
- 20 Hz update

Noise

Gaussian

Correlated

Weighted update

Standard MPPI exponential weighting

Controller output

Steering wheel angle

---

# 6. Steering

Current assumption

DrivMan.Steering.Ang

=

steering wheel angle

Units

radians

Prediction converts

frontWheelAngle =
steeringWheelAngle / steeringRatio

Current steering ratio

16

Need to verify actual steering ratio from vehicle configuration later.

---

# 7. MPPI Cost Function

Running cost

- lateral tracking error
- heading error
- steering magnitude
- steering change

Terminal cost

- lateral error
- heading error

Exploration term

Standard MPPI path-integral exploration

Negative total costs are possible.

Negative cost is NOT an error.

---

# 8. Current Reference

Current reference path

Sinusoidal weave

Purpose

Verification only.

The sine path exists to

- verify controller
- compare controllers
- tune MPPI

Later

Replace sine reference with lane-center tracking.

---

# 9. Object Sensor Obstacle

Current implementation

Mounted Object Sensor

VehSensor_0 / OB00

Up to eight detected objects ahead supply dx and dy.

Detected objects are stored nearest-first when dtct == 1 and dx > 0.

MPPI applies the existing obstacle penalty to every valid object in the list.

No hard-coded obstacle position remains.

---

# 10. User.Out Mapping

User.Out[0]

Target lateral position

User.Out[1]

Actual lateral position

User.Out[2]

Lateral error

User.Out[3]

Heading error

User.Out[4]

Applied steering command

User.Out[5]

Raw MPPI steering request

User.Out[6]

Best rollout cost

User.Out[7]

Controller mode

1 = MPPI

0 = fallback

2 = recovery

User.Out[8]

Reserved

Obstacle longitudinal distance

Nearest obstacle dx, or 999 when none are detected

User.Out[9]

Reserved

Obstacle lateral distance

Nearest obstacle dy, or 0 when none are detected

---

# 11. Logging

Current controller log

MPPI steering:

target

actual

command

cost

mode

Examples

mode=MPPI

controller active

mode=fallback

fallback controller active

mode=RECOVERY

low-speed recovery active after the vehicle has previously exceeded 1 m/s

Speed state logging

STARTUP = vehicle has not yet exceeded 1 m/s

MOVING = vehicle has exceeded 1 m/s and is currently at or above 1 m/s

STALLED = vehicle previously exceeded 1 m/s and is currently below 1 m/s

Obstacle logging includes obstacleCount, nearest dx/dy/id, and the concise list of object IDs.

---

# 12. CarMaker Notes

Active controller

User_DrivMan_Calc()

User_VehicleControl_Calc()

debug only

User.Out

Registered through

DDefDouble()

No copying through User_Out() required.

IPGControl reads User.Out directly.

---

# 13. Object Sensor Notes

Goal

Replace hard-coded obstacle.

Header

Vehicle/Sensor_Object.h

Initialization

Find sensor once.

Never every cycle.

Functions

ObjectSensor_FindIndexForName()

returns sensor index

ObjectSensor_GetByIndex()

returns tObjectSensor*

ObjectSensor_GetObjectByObjId()

returns tObjectSensorObj*

Sensor object

Contains

ObsvObjects

Array of observed traffic object IDs.

Useful object information

- detected flag
- relative x
- relative y
- nearest point
- width
- length
- object ID

Plan

Traffic Object

↓

Object Sensor OB00

↓

Up to eight nearest detected objects ahead

↓

Convert

dx

dy

↓

MPPI obstacle

---

# 14. Planned Sensor Integration

Current

Object Sensor

VehSensor_0 / OB00

Steps

1.

Initialize sensor once during TestRun.

2.

Read observed objects.

3.

Collect detected objects ahead, sort nearest-first, and limit the list to eight.

4.

Convert

dx

dy

to obstacle position.

5.

Use every valid detected object in the MPPI obstacle-cost loop.

---

# 15. Coding Rules

Keep changes small.

One feature at a time.

Preserve working controller.

Never rewrite working MPPI.

Keep fallback controller.

Prefer helper functions.

Do not perform sensor lookup every simulation cycle.

---

# 16. MPPI Parameters

Current

Samples

256

Prediction horizon

2 seconds

Optimization frequency

20 Hz

Prediction model

Kinematic bicycle

Current tuning

Running cost

12 × lateral error²

80 × heading error²

0.2 × steering²

0.5 × steering change²

Terminal cost

30 × lateral error²

120 × heading error²

---

# 17. Validation Checklist

Current

✓ Controller builds

✓ MPPI active

✓ Fallback works

✓ Tracking works

✓ Hard-coded obstacle avoidance works

Next

□ Sensor detects obstacle

□ MPPI avoids sensor obstacle

□ Multiple obstacles

□ Dynamic obstacles

---

# 18. Performance Metrics

Eventually compare

Proportional controller

vs

MPPI

Metrics

RMS lateral error

Maximum lateral error

Maximum steering angle

Steering RMS

Steering smoothness

Obstacle clearance

Computation time

---

# 19. Current Status Summary

The MPPI implementation is functioning.

The controller

- predicts vehicle motion,
- samples steering trajectories,
- evaluates trajectory cost,
- updates the nominal steering sequence,
- uses the mounted Object Sensor as its only obstacle source,
- evaluates up to eight detected obstacles per rollout state,
- returns toward lane center.

The sensor obstacle cost is disabled when no forward object is detected.

Low speed after prior movement enters RECOVERY mode rather than permanently switching to fallback steering.

2026-07-16
- Sensor-based MPPI obstacle avoidance succeeded.
- Object Sensor VehSensor_0 detected traffic vehicle.
- MPPI avoided obstacle and returned to lane.
- Disabled IPGDriver “Consider traffic” so longitudinal control does not stop behind obstacle.
