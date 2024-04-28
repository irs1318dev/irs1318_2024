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
    ArmShoulderWristPosition1,
    ArmShoulderWristPosition2,
    ArmShoulderWristPosition3,
    ArmShoulderWristPosition4,
    ArmShoulderWristPosition5,
    ArmShoulderWristPosition6,
    ArmShoulderWristPosition7,
    ArmShoulderWristPosition8,
    ArmShoulderWristPosition9,
    ArmShoulderWristPosition10,
    ArmShoulderWristPosition11,
    ResetArm,

    // EndEffector operations:
    ShooterSpinSpeed1,
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
