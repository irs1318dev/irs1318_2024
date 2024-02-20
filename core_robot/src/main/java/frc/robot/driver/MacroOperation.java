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

    // Arm operations:
    ArmShoulderPosition1,
    ArmShoulderPosition2,
    ArmShoulderPosition3,
    ArmShoulderPosition4,
    ArmWristPosition1,
    ArmWristPosition2,
    ArmWristPosition3,
    ArmWristPosition4,
    ArmWristPosition5,
    ResetArm,

    // EndEffector operations:
    ShooterSpinSpeed1,
    ShooterSpinSpeed2,

    // Shooter Operations
    ShootNote,

    // Climber
    ClimbMacro,

    // Vision operations

    // Path testing:
    FollowPathTest1,
    FollowPathTest2,
    FollowPathTest3,
    FollowPathTest4,
}
