package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,
    PositionResetRobotLevel,

    // Driver interaction operations
    ForceLightDriverRumble,

    // Vision operations:
    VisionForceDisable,
    VisionDisableStream,
    VisionEnableAprilTagProcessing,

    // DriveTrain operations:
    DriveTrainSlowMode,
    DriveTrainPathMode,
    DriveTrainSteerMode,
    DriveTrainMaintainPositionMode,
    DriveTrainReset,
    DriveTrainEnableFieldOrientation,
    DriveTrainDisableFieldOrientation,
    DriveTrainUseRobotOrientation,
    DriveTrainEnableMaintainDirectionMode,
    DriveTrainDisableMaintainDirectionMode,
    DriveTrainResetXYPosition,
    DriveTrainIgnoreSlewRateLimitingMode,

    // EndEffector operations:
    IntakeIn,
    IntakeOut,
    IntakeForceStop,
    IntakeForceInOn,
    IntakeForceInOff,
    ShooterFeedRing,
    ShooterEnableShootAnywayMode,
    ShooterDisableShootAnywayMode,

    // Arm operations:
    ArmEnableSimpleMode,
    ArmDisableSimpleMode,
    ArmForceReset,

    // Climber operations:
    ClimberServoUp,
    ClimberServoDown,
}
