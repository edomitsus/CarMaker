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
constexpr double ObstacleLateralRadius = 1.75;             /* avoidance envelope [m] */
constexpr double PreferredPassingLateralPosition = 2.25;  /* deterministic left pass [m] */
constexpr double RoadSafetyMargin = 0.5;                  /* edge approach margin [m] */
constexpr double DefaultRoadWidth = 6.0;                  /* conservative eval fallback [m] */

tRoadEval *MppiRoadEval = nullptr;
double RoadEvaluationSOffset = 0.0;

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

constexpr double SteeringLimit = 15.0 * Pi / 180.0;      /* steering-wheel angle [rad] */
constexpr double SteeringRateLimit = 30.0 * Pi / 180.0;  /* [rad/s] */
constexpr double SteeringAccelerationLimit = 2.0;        /* [rad/s^2] */
constexpr double RecoverySteeringLimit = 5.0 * Pi / 180.0; /* gentle stalled recovery [rad] */

enum class SpeedState {
    Startup,
    Moving,
    Stalled
};

enum class ControllerMode {
    Fallback,
    Mppi,
    Recovery
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
constexpr bool MppiEnabled = false;

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

double
ObstacleCost(double roadDistance, double lateralPosition)
{
    double totalCost = 0.0;

    for (int i = 0; i < SensorObstacleCount; ++i) {
        const Obstacle &obstacle = SensorObstacles[i];
        if (!obstacle.valid) {
            continue;
        }

        const double obstacleRoadDistance =
            SensorObstacleReferenceRoadDistance + obstacle.dx;
        const double obstacleLateralPosition =
            SensorObstacleReferenceLateralPosition + obstacle.dy;
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

double
RoadBoundaryCost(double roadDistance, double lateralPosition)
{
    double roadWidth = DefaultRoadWidth;
    if (MppiRoadEval != nullptr) {
        tRoadRouteIn rIn{};
        tRoadRouteOut rOut{};
        rIn.st[0] = RoadEvaluationSOffset + roadDistance;
        rIn.st[1] = lateralPosition;

        if (RoadRouteEval(MppiRoadEval, nullptr, RIT_ST, &rIn, &rOut) == ROAD_Ok
            && std::isfinite(rOut.width[0]) && rOut.width[0] > 0.0
            && std::isfinite(rOut.width[1]) && rOut.width[1] > 0.0) {
            const double evaluatedRoadWidth = rOut.width[0] + rOut.width[1];
            if (std::isfinite(evaluatedRoadWidth) && evaluatedRoadWidth > 0.0) {
                roadWidth = evaluatedRoadWidth;
            }
        }
    }

    const double halfRoadWidth = 0.5 * roadWidth;
    const double softLimit = fmax(0.0, halfRoadWidth - RoadSafetyMargin);
    const double hardLimit = halfRoadWidth;
    const double absoluteLateralPosition = fabs(lateralPosition);
    double cost = 0.0;
    if (absoluteLateralPosition > softLimit) {
        const double excess = absoluteLateralPosition - softLimit;
        cost += 500.0 * excess * excess;
    }
    if (absoluteLateralPosition > hardLimit) {
        const double excess = absoluteLateralPosition - hardLimit;
        cost += 5000.0 * (1.0 + excess * excess);
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

private:
    std::array<double, MppiHorizon> NominalControl;
    std::array<std::array<double, MppiHorizon>, MppiSamples> Noise;
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
    static unsigned int logCounter = 0;

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
        mppiController.Reset();
        logCounter = 0;
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

    constexpr double FallbackLateralGain = 0.12;
    constexpr double FallbackHeadingGain = 1.5;

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
    const double headingError = WrapAngle(
        desiredHeadingRelativeToRoad - headingRelativeToRoad);

    const double lateralError = desiredLateralPosition - Vehicle.Road.Path.tRoad;
    const double fallbackSteering = Clamp(
        FallbackLateralGain * lateralError + FallbackHeadingGain * headingError,
        -SteeringLimit, SteeringLimit);

    if (speedState == SpeedState::Moving) {
        mppiUpdateTimer += dt;
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
               obstacle avoidance" task). fallbackSteering stays an
               absolute safety-net value, unchanged, for when MPPI itself
               is invalid. */
            mppiRequestedSteering = result.Valid
                ? ipgBaselineSteering + result.Command : fallbackSteering;
            if (result.Valid) {
                hasLastValidMppiCommand = true;
                lastValidMppiCommand = result.Command;
                previousMppiDelta = result.Command;
            }
        }
        controllerMode = mppiValid ? ControllerMode::Mppi : ControllerMode::Fallback;
    } else if (speedState == SpeedState::Stalled) {
        mppiValid = false;
        const double recoverySource = hasLastValidMppiCommand
            ? lastValidMppiCommand : fallbackSteering;
        mppiRequestedSteering = Clamp(
            recoverySource, -RecoverySteeringLimit, RecoverySteeringLimit);
        controllerMode = ControllerMode::Recovery;
    } else {
        mppiValid = false;
        mppiRequestedSteering = fallbackSteering;
        controllerMode = ControllerMode::Fallback;
    }

    const double previousSteeringCommand = steeringCommand;
    const double maximumSteeringStep = SteeringRateLimit * dt;
    const double steeringError = mppiRequestedSteering - steeringCommand;
    steeringCommand += Clamp(steeringError, -maximumSteeringStep, maximumSteeringStep);

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
        : (controllerMode == ControllerMode::Recovery ? 2.0 : 0.0);
    User.Out[8] = NearestObstacleDx();
    User.Out[9] = NearestObstacleDy();

    if (logCounter++ % 1000 == 0) {
        char obstacleIds[96] = "none";
        if (SensorObstacleCount > 0) {
            size_t offset = 0;
            obstacleIds[0] = '\0';
            for (int i = 0; i < SensorObstacleCount; ++i) {
                const int written = snprintf(
                    obstacleIds + offset, sizeof(obstacleIds) - offset,
                    i == 0 ? "%d" : ",%d", SensorObstacles[i].objId);
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
            "tRoad=%.3f m curvature=%.5f 1/m baseline=%.2f deg delta=%.2f deg\n",
            SpeedStateName(speedState), SensorObstacleCount,
            NearestObstacleDx(), NearestObstacleDy(), nearestObjectId, obstacleIds,
            steeringCommand * 180.0 / Pi, mppiBestCost,
            ControllerModeName(controllerMode),
            Vehicle.Road.Path.tRoad, currentRoadCurvature,
            ipgBaselineSteering * 180.0 / Pi, previousMppiDelta * 180.0 / Pi);
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
        Log("BUILD STAMP: MPPI_TOGGLE_V11 (built %s %s)\n", __DATE__, __TIME__);
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
