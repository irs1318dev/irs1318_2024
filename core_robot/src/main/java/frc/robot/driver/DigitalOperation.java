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
    VisionEnableStream,
    VisionFindSpeakerAprilTagRear,
    VisionFindSpeakerAprilTagFront,
    VisionFindStageAprilTagsRear,
    VisionFindStageAprilTagsFront,
    VisionFindAmpAprilTagRear,
    VisionFindAmpAprilTagFront,
    VisionFindAnyAprilTagRear,
    VisionFindAnyAprilTagFront,
    VisionFindSourceAprilTagsFront,
    VisionFindAbsolutePosition,

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
    IntakeOutSlow,
    IntakeForceOnAndIntakeIn,
    IntakeForceStop,
    IntakeForceSpinOn,
    IntakeForceSpinOff,
    ShooterFeedRing,
    ShooterShootAnywayMode,

    // Arm operations:
    ArmEnableSimpleMode,
    ArmDisableSimpleMode,
    ArmStop,
    ArmForceReset,
    ArmEnableThroughBore,
    ArmDisableThroughBore,
    ArmEnableProtection,
    ArmDisableProtection,
    ArmSlowMode,

    // Climber operations:
    ClimberServoUp,
    ClimberServoDown,
    ClimberWinchUp,
    ClimberWinchDown,
}
