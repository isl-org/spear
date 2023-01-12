//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "OpenBot/OpenBotWheel.h"

#include "CoreUtils/CompilerWarningUtils.h"
#include "CoreUtils/Config.h"

BEGIN_IGNORE_COMPILER_WARNINGS
UOpenBotWheel::UOpenBotWheel()
{
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CollisionMeshObj(TEXT("/Engine/EngineMeshes/Cylinder"));
    CollisionMesh = CollisionMeshObj.Object;

    bAffectedByHandbrake       = Config::getValue<bool>({"OPENBOT", "OPENBOT_WHEEL", "AFFECTED_BY_HANDBRAKE"});
    DampingRate                = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "DAMPING_RATE"});
    LatStiffMaxLoad            = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "LAT_STIFF_MAX_LOAD"});
    LatStiffValue              = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "LAT_STIFF_VALUE"});
    LongStiffValue             = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "LONG_STIFF_VALUE"});
    Mass                       = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "MASS"});
    MaxBrakeTorque             = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "MAX_BRAKE_TORQUE"});
    MaxHandBrakeTorque         = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "MAX_HAND_BRAKE_TORQUE"});
    ShapeRadius                = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "SHAPE_RADIUS"});
    ShapeWidth                 = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "SHAPE_WIDTH"});
    SteerAngle                 = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "STEER_ANGLE"});
    SuspensionMaxRaise         = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "SUSPENSION_MAX_RAISE"});
    SuspensionMaxDrop          = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "SUSPENSION_MAX_DROP"});
    SuspensionForceOffset      = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "SUSPENSION_FORCE_OFFSET"});
    SuspensionNaturalFrequency = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "SUSPENSION_NATURAL_FREQUENCY"});
    SuspensionDampingRatio     = Config::getValue<float>({"OPENBOT", "OPENBOT_WHEEL", "SUSPENSION_DAMPING_RATIO"});
    SweepType                  = EWheelSweepType::SimpleAndComplex;
    bAutoAdjustCollisionSize   = true; // Set to true if you want to scale the wheels manually 
}
END_IGNORE_COMPILER_WARNINGS
