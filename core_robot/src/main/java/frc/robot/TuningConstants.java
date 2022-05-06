package frc.robot;

import frc.robot.driver.DigitalOperation;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = true;
    public static boolean THROW_EXCEPTIONS = !TuningConstants.COMPETITION_ROBOT;
    public static boolean LOG_EXCEPTIONS = true;

    public static final boolean EXPECT_UNUSED_JOYSTICKS = true;

    //================================================== Magic Values ==============================================================

    public static final double MAGIC_NULL_VALUE = -1318.0;
    public static final double PERRY_THE_PLATYPUS = 0.0;
    public static final double ENDGAME_START_TIME = 30.0;
    public static final double ENDGAME_CLIMB_TIME = 5.0;

    //================================================== Logging  ==============================================================

    public static final int CALENDAR_YEAR = 2022;
    public static final boolean LOG_TO_FILE = TuningConstants.COMPETITION_ROBOT;
    public static final boolean LOG_FILE_ONLY_COMPETITION_MATCHES = true;
    public static final long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static final int LOG_FLUSH_THRESHOLD = 25;

    //================================================= Vision ======================================================

    // Acceptable vision centering range values in degrees
    public static final double MAX_VISION_CENTERING_RANGE_DEGREES = 5.0;

    // How long the robot system must remain centered on the target when using time
    public static final double VISION_CENTERING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double VISION_STATIONARY_CENTERING_PID_KP = 0.025;
    public static final double VISION_STATIONARY_CENTERING_PID_KI = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KD = 0.01;
    public static final double VISION_STATIONARY_CENTERING_PID_KF = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KS = 1.0;
    public static final double VISION_STATIONARY_CENTERING_PID_MIN = -0.4;
    public static final double VISION_STATIONARY_CENTERING_PID_MAX = 0.4;

    // PID settings for Centering the robot on a vision target
    public static final double VISION_MOVING_CENTERING_PID_KP = 0.01;
    public static final double VISION_MOVING_CENTERING_PID_KI = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KD = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KF = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KS = 1.0;
    public static final double VISION_MOVING_CENTERING_PID_MIN = -0.3;
    public static final double VISION_MOVING_CENTERING_PID_MAX = 0.3;

    // PID settings for Advancing the robot towards a vision target
    public static final double VISION_ADVANCING_PID_KP = 0.015;
    public static final double VISION_ADVANCING_PID_KI = 0.0;
    public static final double VISION_ADVANCING_PID_KD = 0.0;
    public static final double VISION_ADVANCING_PID_KF = 0.0;
    public static final double VISION_ADVANCING_PID_KS = 1.0;
    public static final double VISION_ADVANCING_PID_MIN = -0.3;
    public static final double VISION_ADVANCING_PID_MAX = 0.3;

    // PID settings for Advancing the robot quickly towards a vision target
    public static final double VISION_FAST_ADVANCING_PID_KP = 0.15;
    public static final double VISION_FAST_ADVANCING_PID_KI = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KD = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KF = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KS = 1.0;
    public static final double VISION_FAST_ADVANCING_PID_MIN = -0.45;
    public static final double VISION_FAST_ADVANCING_PID_MAX = 0.45;

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 500;

    //================================================== Indicator Lights ========================================================

    public static final double INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE = 3.0;

    public static final int CANDLE_LED_COUNT = 8;
    public static final int LED_STRIP_LED_COUNT = 60; // 60 LEDs per meter-long strip from CTRE
    public static final int CANDLE_TOTAL_NUMBER_LEDS = TuningConstants.CANDLE_LED_COUNT; //+ TuningConstants.LED_STRIP_LED_COUNT; // * 2;

    public static final int CANDLE_ANIMATION_SLOT_1 = 0;
    public static final int CANDLE_ANIMATION_SLOT_2 = 1;

    // IRS1318 Purple color
    public static final int INDICATOR_PURPLE_RED = 101;
    public static final int INDICATOR_PURPLE_GREEN = 34;
    public static final int INDICATOR_PURPLE_BLUE = 129;
    public static final int INDICATOR_PURPLE_WHITE = 0;

    // No color
    public static final int INDICATOR_OFF_COLOR_RED = 0;
    public static final int INDICATOR_OFF_COLOR_GREEN = 0;
    public static final int INDICATOR_OFF_COLOR_BLUE = 0;
    public static final int INDICATOR_OFF_COLOR_WHITE = 0;

    // Bright Red color
    public static final int INDICATOR_RED_COLOR_RED = 255;
    public static final int INDICATOR_RED_COLOR_GREEN = 0;
    public static final int INDICATOR_RED_COLOR_BLUE = 0;
    public static final int INDICATOR_RED_COLOR_WHITE = 0;

    // Bright Yellow color
    public static final int INDICATOR_YELLOW_COLOR_RED = 255;
    public static final int INDICATOR_YELLOW_COLOR_GREEN = 255;
    public static final int INDICATOR_YELLOW_COLOR_BLUE = 0;
    public static final int INDICATOR_YELLOW_COLOR_WHITE = 0;

    // Bright Green color
    public static final int INDICATOR_GREEN_COLOR_RED = 0;
    public static final int INDICATOR_GREEN_COLOR_GREEN = 255;
    public static final int INDICATOR_GREEN_COLOR_BLUE = 0;
    public static final int INDICATOR_GREEN_COLOR_WHITE = 0;

    public static final double COMPRESSOR_FILL_RATE = 10.0;
    public static final double COMPRESSOR_ENOUGH_PRESSURE = 110.0;

    //================================================== DriveTrain ==============================================================

    public static final boolean DRIVETRAIN_USE_ODOMETRY = true;
    public static final boolean DRIVETRAIN_RESET_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START = true;

    public static final double DRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = -124.365;
    public static final double DRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = 51.943;
    public static final double DRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = -128.848;
    public static final double DRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = -128.672;

    // Position PID (angle) per-module
    public static final double DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS = HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_DEGREE;

    public static final double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KP = 0.5;
    public static final double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KP = 0.5;
    public static final double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KP = 0.5;
    public static final double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KP = 0.5;
    public static final double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KF = 0.0;

    // Velocity PID (drive) per-module
    public static final double DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS = 17000.0; // 21400 was highest speed at full throttle FF on blocks. this is #ticks / 100ms

    public static final double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KP = 0.09;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static final double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KP = 0.09;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static final double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KP = 0.09;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static final double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KP = 0.09;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static final double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KP = 0.1;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_OMEGA_MAX_OUTPUT = 4.0;
    public static final double DRIVETRAIN_OMEGA_MIN_OUTPUT = -4.0;

    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KP = 0.1;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_OMEGA_MAX_OUTPUT = 4.0;
    public static final double DRIVETRAIN_PATH_OMEGA_MIN_OUTPUT = -4.0;

    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KP = 0.0; // 1.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_X_MAX_OUTPUT = 10.0;
    public static final double DRIVETRAIN_PATH_X_MIN_OUTPUT = -10.0;

    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KP = 0.0; // 1.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_Y_MAX_OUTPUT = 10.0;
    public static final double DRIVETRAIN_PATH_Y_MIN_OUTPUT = -10.0;

    public static final boolean DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION = 11.0;
    public static final boolean DRIVETRAIN_DRIVE_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_CURRENT_MAX = 30.0;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_CURRENT = 40.0;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_DURATION = 100.0;

    public static final boolean DRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double DRIVETRAIN_STEER_VOLTAGE_COMPENSATION = 11.0;
    public static final boolean DRIVETRAIN_STEER_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double DRIVETRAIN_STEER_SUPPLY_CURRENT_MAX = 30.0;
    public static final double DRIVETRAIN_STEER_SUPPLY_TRIGGER_CURRENT = 40.0;
    public static final double DRIVETRAIN_STEER_SUPPLY_TRIGGER_DURATION = 100.0;

    public static final int DRIVETRAIN_SENSOR_FRAME_PERIOD_MS = 10;
    public static final int DRIVETRAIN_PID_FRAME_PERIOD_MS = 100;

    public static final boolean DRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY = true;
    public static final double DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA = 0.001;
    public static final double DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA = 0.25;

    public static final double DRIVETRAIN_DEAD_ZONE_TURN = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_VELOCITY = 0.15;
    public static final double DRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.15;

    public static final double DRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static final double DRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    public static final double DRIVETRAIN_MAX_VELOCITY = TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND; // max velocity in inches per second
    public static final double DRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static final double DRIVETRAIN_TURN_GOAL_VELOCITY = 10.0; // degrees per second for turn goal
    public static final double DRIVETRAIN_TURN_SCALE = 4.0; // radians per second
    public static final double DRIVETRAIN_STATIONARY_VELOCITY = 0.1;
    public static final double DRIVETRAIN_TURN_APPROXIMATION_STATIONARY = 2.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_TURN_APPROXIMATION = 1.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static final double DRIVETRAIN_MAX_PATH_TURN_VELOCITY = 180.0; // in degrees per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.80 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.75 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second per second
}
