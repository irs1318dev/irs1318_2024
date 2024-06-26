package frc.robot;

import frc.lib.robotprovider.PowerDistributionModuleType;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    // We expect the following to be true.  Change INVERT_*_AXIS to true if any of the following are not met:
    // 1. forwards/up on a joystick is positive, backwards/down is negative.
    // 2. right on a joystick is positive, left on a joystick is negative.
    // 3. pressed on a trigger is positive, released is negative/zero.
    public static final boolean INVERT_XBONE_LEFT_X_AXIS = false;
    public static final boolean INVERT_XBONE_RIGHT_X_AXIS = false;
    public static final boolean INVERT_XBONE_LEFT_Y_AXIS = true;
    public static final boolean INVERT_XBONE_RIGHT_Y_AXIS = true;
    public static final boolean INVERT_XBONE_LEFT_TRIGGER = false;
    public static final boolean INVERT_XBONE_RIGHT_TRIGGER = false;

    public static final boolean INVERT_PS4_LEFT_X_AXIS = false;
    public static final boolean INVERT_PS4_RIGHT_X_AXIS = false;
    public static final boolean INVERT_PS4_LEFT_Y_AXIS = true;
    public static final boolean INVERT_PS4_RIGHT_Y_AXIS = true;
    public static final boolean INVERT_PS4_LEFT_TRIGGER = false;
    public static final boolean INVERT_PS4_RIGHT_TRIGGER = false;

    public static final boolean INVERT_THROTTLE_AXIS = true;
    public static final boolean INVERT_TRIGGER_AXIS = false;

    public static final int POWER_DISTRIBUTION_CAN_ID = 1;
    public static final PowerDistributionModuleType POWER_DISTRIBUTION_TYPE = PowerDistributionModuleType.PowerDistributionHub;

    public static final double REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MIN = 1.0 / 1024.0;
    public static final double REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MAX = 1023.0 / 1024.0;

    public static final String CANIVORE_NAME = "CANIVORE1"; // Module A

    //================================================== IMU ==============================================================

    public static final int PIGEON_IMU_CAN_ID = 0;

    //================================================== SDSDriveTrain ==============================================================

    public static final int SDSDRIVETRAIN_DRIVE_MOTOR_1_CAN_ID = 1;
    public static final int SDSDRIVETRAIN_STEER_MOTOR_1_CAN_ID = 2;
    public static final int SDSDRIVETRAIN_DRIVE_MOTOR_2_CAN_ID = 3;
    public static final int SDSDRIVETRAIN_STEER_MOTOR_2_CAN_ID = 4;
    public static final int SDSDRIVETRAIN_DRIVE_MOTOR_3_CAN_ID = 5;
    public static final int SDSDRIVETRAIN_STEER_MOTOR_3_CAN_ID = 6;
    public static final int SDSDRIVETRAIN_DRIVE_MOTOR_4_CAN_ID = 7;
    public static final int SDSDRIVETRAIN_STEER_MOTOR_4_CAN_ID = 8;

    public static final int SDSDRIVETRAIN_ABSOLUTE_ENCODER_1_CAN_ID = 1;
    public static final int SDSDRIVETRAIN_ABSOLUTE_ENCODER_2_CAN_ID = 2;
    public static final int SDSDRIVETRAIN_ABSOLUTE_ENCODER_3_CAN_ID = 3;
    public static final int SDSDRIVETRAIN_ABSOLUTE_ENCODER_4_CAN_ID = 4;

    //================================================== EndEffector ==============================================================

    public static final int INTAKE_THROUGHBEAM_ANALOG_INPUT = 0;

    public static final int INTAKE_MOTOR_CAN_ID = 21;
    public static final int SHOOTER_NEAR_FLYWHEEL_MOTOR_CAN_ID = 22;
    public static final int SHOOTER_FAR_FLYWHEEL_MOTOR_CAN_ID = 23;

    //================================================== Arm =========================================================================

    public static final int ARM_SHOULDER_MOTOR_CAN_ID = 12;
    public static final int ARM_SHOULDER_FOLLOWER_MOTOR_CAN_ID = 13;
    public static final int ARM_WRIST_MOTOR_CAN_ID = 14;

    public static final boolean ARM_WRIST_LIMIT_SWITCH_ENABLED = true;
    public static final boolean ARM_WRIST_LIMIT_SWITCH_NORMALLY_OPEN = true;

    public static final int ARM_SHOULDER_PDH_CHANNEL = TuningConstants.COMPETITION_ROBOT ? 3 : 6;
    public static final int ARM_SHOULDER_FOLLOWER_PDH_CHANNEL = TuningConstants.COMPETITION_ROBOT ? 4 : 7;
    public static final int ARM_WRIST_PDH_CHANNEL = TuningConstants.COMPETITION_ROBOT ? 16 : 11;
    public static final int ARM_WRIST_ABSOLUTE_ENCODER_DIO_CHANNEL = 1;
    public static final int ARM_SHOULDER_ABSOLUTE_ENCODER_DIO_CHANNEL = 0;

    //================================================= Climber ====================================================================

    public static final int CLIMBER_MOTOR_CAN_ID = 16;
    public static final int CLIMBER_MOTOR_FOLLOWER_CAN_ID = 17;
    public static final int CLIMBER_SERVO_MOTOR_CAN_ID = 1;
    public static final int CLIMBER_LIMIT_SWITCH_DIO_CHANNEL = 2;
}

