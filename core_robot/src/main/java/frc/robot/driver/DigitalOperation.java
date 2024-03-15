package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,
    PositionSwapFieldOrientation,
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
    IntakeForceOnAndIntakeIn,
    IntakeForceStop,
    IntakeForceSpinOn,
    IntakeForceSpinOff,
    ShooterFeedRing,
    ShooterShootAnywayMode,
    // ShooterDisableShootAnywayMode,

    // Arm operations:
    ArmEnableSimpleMode,
    ArmDisableSimpleMode,
    ArmStop,
    ArmForceReset,

    // Climber operations:
    ClimberServoUp,
    ClimberServoDown,
    ClimberWinchUp,
    ClimberWinchDown,
}
