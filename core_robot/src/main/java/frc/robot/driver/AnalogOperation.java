package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum AnalogOperation implements IOperation
{
    PositionStartingAngle,

    // DriveTrain operations:
    DriveTrainMoveForward,
    DriveTrainMoveRight,
    DriveTrainTurnAngleGoal,
    DriveTrainSpinLeft,
    DriveTrainSpinRight,
    DriveTrainRotationA,
    DriveTrainRotationB,
    DriveTrainPathXGoal,
    DriveTrainPathYGoal,
    DriveTrainPathXVelocityGoal,
    DriveTrainPathYVelocityGoal,
    DriveTrainPathAngleVelocityGoal,
    DriveTrainPositionSteer1,
    DriveTrainPositionSteer2,
    DriveTrainPositionSteer3,
    DriveTrainPositionSteer4,
    DriveTrainPositionDrive1,
    DriveTrainPositionDrive2,
    DriveTrainPositionDrive3,
    DriveTrainPositionDrive4,
    DriveTrainStartingXPosition,
    DriveTrainStartingYPosition,

    // EndEffector operations
    FlywheelMotorPower, // set by analog axis
    NearFlywheelVelocityGoal, // Set by Macros
    FarFlywheelVelocityGoal, // Set by Macros 

    // Arm operations
    ArmShoulderPower,
    ArmWristPower,
    ArmShoulderPositionSetpoint,
    ArmWristPositionSetpoint,
    AbsWristAngle,
}
