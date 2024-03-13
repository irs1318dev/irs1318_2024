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
    //ArmWristPosition5,
    ResetArm,

    // EndEffector operations:
    ShooterSpinSpeed1,
    ShooterSpinSpeed2,
    ShootAuto,

    // Shooter Operations
    ShootNote,

    // Climber
    ClimbMacro,
    TrapCombo,

    // Vision operations
    VisionTest,

    // Trap
    AutoTrapScore,
    

    // Path testing:
    FollowPathTest1,
    FollowPathTest2,
    FollowPathTest3,
    FollowPathTest4, 
    
}
