package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDLightBrake,
    PIDHeavyBrake,
    FaceForward,
    FaceBackward,
    FaceLeft,
    FaceRight,
    FaceSource,
    AlignStage,
    AlignAmp,
    FacePassing,

    // Arm operations:
    ArmShoulderWristPositionStartingConfiguration,
    ArmShoulderWristPositionGroundPickup,
    ArmShoulderWristPositionGroundShot,
    ArmShoulderWristPositionUpperShot,
    ArmShoulderWristPosition5,
    ArmShoulderWristPositionPassing,
    ArmShoulderWristPositionAmpOuttake,
    ArmShoulderWristPositionTrapIntermediate,
    ArmShoulderWristPosition9,
    ArmShoulderWristPositionUpperBotDistanceShot,
    ArmShoulderWristPositionTrapDelivery,
    ResetArm,

    // EndEffector operations:
    ShooterSpin,
    IntakeFix,

    // Shooter Operations
    ShootNote,

    // Climber
    ClimbTrap,

    // Vision operations
    VisionShootAim,

    // Path testing:
    FollowPathTest1,
    FollowPathTest2,
    FollowPathTest3,
    FollowPathTest4,
}
