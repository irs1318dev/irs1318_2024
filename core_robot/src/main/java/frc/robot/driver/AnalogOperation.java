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

    FlywheelMotorPower,
    FlywheelVelocityGoal,

    MoveLinearActuatorArm,
    MoveChainArm,
}
