package frc.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = false;
    public static boolean THROW_EXCEPTIONS = !TuningConstants.COMPETITION_ROBOT;
    public static boolean LOG_EXCEPTIONS = true;
    public static double LOOP_DURATION = 0.02; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)
    public static int LOOPS_PER_SECOND = 50; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)

    public static final boolean EXPECT_UNUSED_JOYSTICKS = true;

    //================================================== Magic Values ==============================================================

    public static final double MAGIC_NULL_VALUE = -1318.0;
    public static final double ZERO = 0.0;
    public static final double ENDGAME_START_TIME = 30.0;
    public static final double ENDGAME_CLIMB_TIME = 5.0;

    //================================================== Logging  ==============================================================

    public static final int CALENDAR_YEAR = 2024;
    public static final boolean LOG_TO_FILE = true; // TuningConstants.COMPETITION_ROBOT;
    public static final boolean LOG_FILE_ONLY_COMPETITION_MATCHES = false;
    public static final long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static final int LOG_FLUSH_THRESHOLD = 25;
    public static final boolean USE_LOGGING_FREQUENCY = true; // TuningConstants.COMPETITION_ROBOT;
    public static final int DEFAULT_LOGGING_FREQUENCY = 10; // number of entries to ignore between logging

    //================================================== Autonomous ==============================================================

    public static final boolean TRAJECTORY_FORCE_BUILD = false;

    //================================================= Power ======================================================

    public static final double POWER_OVERCURRENT_TRACKING_DURATION = 5.0; // duration of time to keep track of the average current
    public static final double POWER_OVERCURRENT_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double POWER_OVERCURRENT_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double POWER_OVERCURRENT_THRESHOLD = 140.0;
    public static final double POWER_OVERCURREHT_HIGH_THRESHOLD = 180.0;

    //================================================= Macros/Vision ======================================================

    public static final int APRILTAG_BLUE_SOURCE_RIGHT_ID = 1;
    public static final int APRILTAG_BLUE_SOURCE_LEFT_ID = 2;
    public static final int APRILTAG_RED_SPEAKER_OFFCENTER_ID = 3;
    public static final int APRILTAG_RED_SPEAKER_CENTER_ID = 4;
    public static final int APRILTAG_RED_AMP_ID = 5;
    public static final int APRILTAG_BLUE_AMP_ID = 6;
    public static final int APRILTAG_BLUE_SPEAKER_CENTER_ID = 7;
    public static final int APRILTAG_BLUE_SPEAKER_OFFCENTER_ID = 8;
    public static final int ARPILTAG_RED_SOURCE_RIGHT_ID = 9;
    public static final int APRILTAG_RED_SOURCE_LEFT_ID = 10;
    public static final int APRILTAG_RED_STAGE_LEFT_ID = 11;
    public static final int APRILTAG_RED_CENTER_STAGE_ID = 12;
    public static final int APRILTAG_RED_STAGE_RIGHT_ID = 13;
    public static final int APRILTAG_BLUE_CENTER_STAGE_ID = 14;
    public static final int APRILTAG_BLUE_STAGE_RIGHT_ID = 15;
    public static final int APRILTAG_BLUE_STAGE_LEFT_ID = 16;

    // Finding AprilTags to determine if theres enough valid data to translate 
    public static final int TAGS_MISSED_THRESHOLD = 30;
    public static final int TAGS_FOUND_THRESHOLD = 5;
    public static final double ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE = 1.0; // in inches

    // Acceptable vision centering range values in degrees
    public static final double MAX_PID_TURNING_RANGE_DEGREES = 7.0;

    // How long the robot system must remain centered on the target when using time
    public static final double PID_TURNING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;
    public static final double MAX_VISION_ACCEPTABLE_STRAFE_DISTANCE = 0.7;

    // Acceptable vision distance from tape in angles 
    public static final double MAX_VISION_ACCEPTABLE_MOVING_RR_ANGLE_ERROR = 4.0;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double STATIONARY_PID_TURNING_PID_KP = 0.027;
    public static final double STATIONARY_PID_TURNING_PID_KI = 0.0;
    public static final double STATIONARY_PID_TURNING_PID_KD = 0.01;
    public static final double STATIONARY_PID_TURNING_PID_KF = 0.0;
    public static final double STATIONARY_PID_TURNING_PID_KS = 1.0;
    public static final double STATIONARY_PID_TURNING_PID_MIN = -0.4;
    public static final double STATIONARY_PID_TURNING_PID_MAX = 0.4;

    // PID settings for rotating the robot based on a vision target while in-motion
    public static final double VISION_MOVING_TURNING_PID_KP = 0.012;
    public static final double VISION_MOVING_TURNING_PID_KI = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KD = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KF = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KS = 1.0;
    public static final double VISION_MOVING_TURNING_PID_MIN = -0.3;
    public static final double VISION_MOVING_TURNING_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_MOVING_PID_KP = 0.015;
    public static final double VISION_MOVING_PID_KI = 0.0;
    public static final double VISION_MOVING_PID_KD = 0.0;
    public static final double VISION_MOVING_PID_KF = 0.0;
    public static final double VISION_MOVING_PID_KS = 1.0;
    public static final double VISION_MOVING_PID_MIN = -0.3;
    public static final double VISION_MOVING_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_AT_TRANSLATION_X_PID_KP = 0.023;
    public static final double VISION_AT_TRANSLATION_X_PID_KI = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KD = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KF = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KS = 1.0;
    public static final double VISION_AT_TRANSLATION_X_PID_MIN = -0.3;
    public static final double VISION_AT_TRANSLATION_X_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_AT_TRANSLATION_Y_PID_KP = 0.023;
    public static final double VISION_AT_TRANSLATION_Y_PID_KI = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KD = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KF = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KS = 1.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_MIN = -0.3;
    public static final double VISION_AT_TRANSLATION_Y_PID_MAX = 0.3;

    // PID settings for translating the robot slowly based on a vision target
    public static final double VISION_SLOW_MOVING_PID_KP = 0.013;
    public static final double VISION_SLOW_MOVING_PID_KI = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KD = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KF = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KS = 1.0;
    public static final double VISION_SLOW_MOVING_PID_MIN = -0.3;
    public static final double VISION_SLOW_MOVING_PID_MAX = 0.3;

    // PID settings for translating the robot quickly based on a vision target
    public static final double VISION_FAST_MOVING_PID_KP = 0.17;
    public static final double VISION_FAST_MOVING_PID_KI = 0.0;
    public static final double VISION_FAST_MOVING_PID_KD = 0.0;
    public static final double VISION_FAST_MOVING_PID_KF = 0.0;
    public static final double VISION_FAST_MOVING_PID_KS = 1.0;
    public static final double VISION_FAST_MOVING_PID_MIN = -0.45;
    public static final double VISION_FAST_MOVING_PID_MAX = 0.45;

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 500;

    public static final double ORIENTATION_TURN_THRESHOLD = 2.0; // number of degrees off at which point we give up trying to face an angle

    //================================================== Indicator Lights ========================================================

    public static final double INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE = 3.0;

    public static final int CANDLE_LED_START = 0;
    public static final int CANDLE_LED_COUNT = 8;
    public static final int LED_STRIP_LED_START = TuningConstants.CANDLE_LED_COUNT;
    public static final int LED_STRIP_LED_COUNT = 60; // 60 LEDs per meter-long strip from CTRE
    public static final int CANDLE_TOTAL_NUMBER_LEDS = TuningConstants.CANDLE_LED_COUNT + TuningConstants.LED_STRIP_LED_COUNT;

    public static final int CANDLE_ANIMATION_SLOT_0 = 0;
    public static final int CANDLE_ANIMATION_SLOT_1 = 1;
    public static final int CANDLE_ANIMATION_SLOT_2 = 2;
    public static final int CANDLE_ANIMATION_SLOT_3 = 3;

    // IRS1318 Purple color
    public static final int INDICATOR_PURPLE_COLOR_RED = 101;
    public static final int INDICATOR_PURPLE_COLOR_GREEN = 34;
    public static final int INDICATOR_PURPLE_COLOR_BLUE = 129;
    public static final int INDICATOR_PURPLE_COLOR_WHITE = 0;

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

    // Bright Red color
    public static final int INDICATOR_RED_COLOR_RED = 255;
    public static final int INDICATOR_RED_COLOR_GREEN = 0;
    public static final int INDICATOR_RED_COLOR_BLUE = 0;
    public static final int INDICATOR_RED_COLOR_WHITE = 0;

    // Blue
    public static final int INDICATOR_BLUE_COLOR_RED = 0;
    public static final int INDICATOR_BLUE_COLOR_GREEN = 0;
    public static final int INDICATOR_BLUE_COLOR_BLUE = 255;
    public static final int INDICATOR_BLUE_COLOR_WHITE = 0;

    // Orange
    public static final int INDICATOR_ORANGE_COLOR_RED = 255;
    public static final int INDICATOR_ORANGE_COLOR_GREEN = 165;
    public static final int INDICATOR_ORANGE_COLOR_BLUE = 0;
    public static final int INDICATOR_ORANGE_COLOR_WHITE = 0;

    // Rainbow
    public static final int INDICATOR_RAINBOW_BRIGHTNESS = 1;
    public static final double INDICATOR_RAINBOW_SPEED = 0.25;
    public static final boolean INDICATOR_RAINBOW_REVERSE_DIRECTION = false;

    // No color
    public static final int INDICATOR_OFF_COLOR_RED = 0;
    public static final int INDICATOR_OFF_COLOR_GREEN = 0;
    public static final int INDICATOR_OFF_COLOR_BLUE = 0;
    public static final int INDICATOR_OFF_COLOR_WHITE = 0;

    //================================================== SDS DriveTrain ==============================================================

    public static final boolean SDSDRIVETRAIN_STEER_MOTORS_USE_MOTION_MAGIC = true;
    public static final boolean SDSDRIVETRAIN_USE_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION = false;
    public static final double SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP = 0.02;

    public static final boolean SDSDRIVETRAIN_USE_ODOMETRY = true;
    public static final boolean SDSDRIVETRAIN_RESET_ON_ROBOT_START = true;
    public static final boolean SDSDRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START = true;
    public static final boolean SDSDRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START = true;

    public static final double SDSDRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = 0.17212; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = 0.15698; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = -0.28979; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = -0.35327; // rotations

    public static final boolean SDSDRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING = true;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -3.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE = 3.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final boolean SDSDRIVETRAIN_USE_ROTATIONAL_RATE_LIMITING = true;
    public static final double SDSDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -4.0 * TuningConstants.SDSDRIVETRAIN_TURN_SCALE;
    public static final double SDSDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_POSITIVE_RATE = 4.0 * TuningConstants.SDSDRIVETRAIN_TURN_SCALE;

    // Position PID (angle) per-module
    public static final double SDSDRIVETRAIN_STEER_MOTOR_POSITION_PID_KS = HardwareConstants.SDSDRIVETRAIN_STEER_TICKS_PER_DEGREE;

    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KP = 1.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);

    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KP = 0.3333333 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KV= 0.00934579 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KS= 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_CRUISE_VELOC = 100.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_ACCEL = 600.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_JERK = 9999.0;

    // Velocity PID (drive) per-module
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS = 88.0; // RPM ~110.0 was highest speed at full throttle FF on blocks

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KP = 0.02 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KF = 0.00909 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0); // 100% control authority (on blocks)

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KP = 2.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);

    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KP = 0.1;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_OMEGA_MAX_OUTPUT = 5.0;
    public static final double SDSDRIVETRAIN_OMEGA_MIN_OUTPUT = -5.0;

    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KP = 0.1;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_MAX_OUTPUT = 4.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_MIN_OUTPUT = -4.0;

    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KP = 1.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_PATH_X_MAX_OUTPUT = 10.0;
    public static final double SDSDRIVETRAIN_PATH_X_MIN_OUTPUT = -10.0;

    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KP = 1.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_PATH_Y_MAX_OUTPUT = 10.0;
    public static final double SDSDRIVETRAIN_PATH_Y_MIN_OUTPUT = -10.0;

    public static final boolean SDSDRIVETRAIN_USE_OVERCURRENT_ADJUSTMENT = true;
    public static final double SDSDRIVETRAIN_OVERCURRENT_ADJUSTMENT = 0.75;
    public static final double SDSDRIVETRAIN_OVERCURRENT_HIGH_ADJUSTMENT = 0.5;

    public static final boolean SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION = 12.0;
    public static final boolean SDSDRIVETRAIN_DRIVE_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double SDSDRIVETRAIN_DRIVE_SUPPLY_CURRENT_MAX = 35.0;
    public static final double SDSDRIVETRAIN_DRIVE_SUPPLY_TRIGGER_CURRENT = 35.0;
    public static final double SDSDRIVETRAIN_DRIVE_SUPPLY_TRIGGER_DURATION = 0.25;

    public static final boolean SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION = 12.0;
    public static final boolean SDSDRIVETRAIN_STEER_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double SDSDRIVETRAIN_STEER_SUPPLY_CURRENT_MAX = 20.0;
    public static final double SDSDRIVETRAIN_STEER_SUPPLY_TRIGGER_CURRENT = 30.0;
    public static final double SDSDRIVETRAIN_STEER_SUPPLY_TRIGGER_DURATION = 0.1;

    public static final int SDSDRIVETRAIN_FEEDBACK_UPDATE_RATE_HZ = 100;
    public static final int SDSDRIVETRAIN_ERROR_UPDATE_RATE_HZ = 10;

    public static final boolean SDSDRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY = true;
    public static final double SDSDRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA = 0.001;
    public static final double SDSDRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA = 0.36;

    public static final double SDSDRIVETRAIN_EXPONENTIAL = 2.0;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_TURN = 0.1;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_X = 0.1;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_Y = 0.1;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.1;

    public static final double SDSDRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static final double SDSDRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    public static final double SDSDRIVETRAIN_MAX_VELOCITY = TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS * HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND; // max velocity in inches per second
    public static final double SDSDRIVETRAIN_SLOW_MODE_MAX_VELOCITY = 0.3 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // max velocity in inches per second
    public static final double SDSDRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final double SDSDRIVETRAIN_TURN_GOAL_VELOCITY = 10.0; // degrees per second for turn goal
    public static final double SDSDRIVETRAIN_TURN_SCALE = 1.6 * Math.PI; // radians per second
    public static final double SDSDRIVETRAIN_SLOW_MODE_TURN_SCALE = 0.3 * TuningConstants.SDSDRIVETRAIN_TURN_SCALE; // radians per second
    public static final double SDSDRIVETRAIN_STATIONARY_VELOCITY = 0.1;
    public static final double SDSDRIVETRAIN_TURN_APPROXIMATION_STATIONARY = 2.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double SDSDRIVETRAIN_TURN_APPROXIMATION = 1.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double SDSDRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static final double SDSDRIVETRAIN_MAX_PATH_TURN_VELOCITY = 180.0; // in degrees per second
    public static final double SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.60 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.75 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.75 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY = 0.60 * TuningConstants.SDSDRIVETRAIN_MAX_PATH_TURN_VELOCITY; // in degrees per second
    public static final double SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION = 0.75 * TuningConstants.SDSDRIVETRAIN_MAX_PATH_TURN_VELOCITY; // in degrees per second per second
    public static final double SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 1.4; // in inches per second
    public static final double SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 1.25; // in inches per second per second
    public static final double SDSDRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 2.0; // in inches per second
    public static final double SDSDRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 2.0; // in inches per second per second

    //================================================== EndEffector ==============================================================

    public static final boolean INTAKE_MOTOR_INVERT_OUTPUT = true;
    public static final double EFFECTOR_INTAKE_IN_POWER = 0.6;
    public static final double EFFECTOR_INTAKE_OUT_POWER = -0.6;
    public static final double EFFECTOR_INTAKE_FEED_SHOOTER_POWER = 0.9;

    public static final boolean NEAR_SHOOTER_MOTOR_INVERT_SENSOR = false;
    public static final boolean NEAR_SHOOTER_MOTOR_INVERT_OUTPUT = false;

    public static final boolean FAR_SHOOTER_MOTOR_INVERT_SENSOR = false;
    public static final boolean FAR_SHOOTER_MOTOR_INVERT_OUTPUT = false;

    public static final double SHOOTER_NEAR_FLYWHEEL_MAX_VELOCITY = 5300.0; // (RPM)
    public static final double SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KP = 0.00040;
    public static final double SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KI = 0.0;
    public static final double SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KD = 0.015;
    public static final double SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KF = 0.00018868;

    public static final double SHOOTER_FAR_FLYWHEEL_MAX_VELOCITY = 5000.0; // (RPM)
    public static final double SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KP = 0.00032;
    public static final double SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KI = 0.0;
    public static final double SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KD = 0.015;
    public static final double SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KF = 0.0002;

    public static final int FLYWHEEL_STALL_LIMIT = 100;
    public static final int FLYWHEEL_FREE_LIMIT = 100;
    public static final int FLYWHEEL_RPM_LIMIT = 2000;
    public static final int FLYWHEEL_SENSOR_FRAME_PERIOD_MS = 10;

    public static final double INTAKE_THROUGHBEAM_CUTOFF = 2.7;
    public static final double EFFECTOR_OUTTAKE_DURATION = 0.5; // time from when ring no longer between through beams to out of the effector when outtaking
    public static final double EFFECTOR_SHOOTING_DURATION = 1.0;

    public static final double FLYWHEEL_ALLOWABLE_ERROR_RANGE = 500;

    //==================================================== ChainAndSprocketArm ==============================================================

    public static final boolean ARM_SHOULDER_MOTOR_INVERT_OUTPUT = false;
    // public static final boolean ARM_SHOULDER_MOTOR_INVERT_SENSOR = false; // N/A - using built-in encoder

    public static final double ARM_SHOULDER_POWER_STRENGTH = 1.0;
    public static final double ARM_WRIST_POWER_STRENGTH = 0.6;
    public static final double ARM_POWER_EXPONENTIAL = 1.0;
    public static final double ARM_SHOULDER_DEAD_ZONE = 0.1;
    public static final double ARM_WRIST_DEAD_ZONE = 0.1;

    public static final double ARM_SHOULDER_MIN_POSITION = -25.0; // in degrees
    public static final double ARM_SHOULDER_MAX_POSITION = 55.0; // in degrees
    public static final double ARM_WRIST_MIN_POSITION = -110.0; // in degrees
    public static final double ARM_WRIST_MAX_POSITION = 210.0; // in degrees

    // -------------------> SHOULDER POSITIONS <------------------- (all in degrees)
    public static final double ARM_SHOULDER_POSITION_STARTING_CONFIGURATION = -25.0;
    public static final double ARM_WRIST_POSITION_STARTING_CONFIGURATION = -103.0;

    public static final double ARM_SHOULDER_POSITION_LOWER_UNIVERSAL = TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION;
    public static final double ARM_WRIST_POSITION_STOWED = TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION;

    public static final double ARM_WRIST_POSITION_GROUND_PICKUP = 44.0;

    public static final double ARM_SHOULDER_POSITION_TUCKED = 10.0;
    public static final double ARM_WRIST_POSITION_TUCKED_SHOT = 143.0;

    public static final double ARM_SHOULDER_POSITION_SOURCE_PICKUP = TuningConstants.ARM_SHOULDER_POSITION_TUCKED;
    public static final double ARM_WRIST_POSITION_SOURCE_PICKUP = -65.0;

    public static final double ARM_SHOULDER_POSITION_UPPER_UNIVERSAL = 25.0;
    public static final double ARM_WRIST_POSITION_UPPER_UNIVERSAL_SHOT = 75.0;

    public static final double ARM_SHOULDER_POSITION_AMP_SCORE = 45.0;
    public static final double ARM_WRIST_POSITION_AMP_SCORE = 60.0;

    public static final double ARM_SHOULDER_POSITION_INTAKE_FLIPPED = TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL;
    public static final double ARM_WRIST_POSITION_INTAKE_FLIPPED = -90.0;

    public static final double ARM_SHOULDER_POSITION_TRAP_INTERMEDIATE = 50.0; 
    public static final double ARM_WRIST_POSITION_TRAP_INTERMEDIATE = -97.0;

    public static final double ARM_SHOULDER_POSITION_INTAKE_OBTUSE = TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL;
    public static final double ARM_WRIST_POSITION_INTAKE_OBTUSE = 135.0;

    public static final double ARM_WRIST_GOAL_THRESHOLD = 1.0;
    public static final double ARM_SHOULDER_GOAL_THRESHOLD = 1.0;

    public static final double STARTUP_AND_GROUND_PICKUP_WEIGHT = 1.0;
    public static final double STARTUP_AND_SOURCE_PICKUP_WEIGHT = 1.0;
    public static final double STARTUP_AND_UPPER_INTAKE_FLIPPED_WEIGHT = 1.0;
    public static final double SOURCE_PICKUP_AND_UPPER_INTAKE_FLIPPED_WEIGHT = 1.0;
    public static final double UPPER_INTALE_FLIPPED_AND_TRAP_INTER_WEIGHT = 1.0;
    public static final double UPPER_INTALE_FLIPPED_AND_UPPER_UNIV_WEIGHT = 1.0;
    public static final double UPPER_UNIV_AND_GROUND_PICKUP_WEIGHT = 1.0;
    public static final double UPPER_UNIV_AND_OBTUSE_WRIST_WEIGHT = 1.0;
    public static final double AMP_SCORE_AND_OBTUSE_WRIST_WEIGHT = 1.0;
    public static final double OBTUSE_WRIST_AND_GROUND_PICKUP_WEIGHT = 1.0;
    public static final double OBTUSE_WRIST_AND_TUCKED_WEIGHT = 1.0;
    public static final double GROUND_PICKUP_TO_TUCKED_WEIGHT = 1.0;
    public static final double TUCKED_TO_TUCKED_GROUND_TRANS_WEIGHT = 1.0;
    public static final double TUCCKED_GROUND_TRANS_TO_GROUND_PICKUP_WEIGHT = 1.0;

    public static final double ARM_SHOULDER_WEIGHT_MULTIPLIER = 1.0;
    public static final double ARM_WRIST_WEIGHT_MULTIPLIER = 1.0;

    // ----------------> OTHER THINGS <----------------

    public static final boolean ARM_SHOULDER_MOTOR_FOLLOWER_INVERT_OUTPUT = false;

    public static final boolean ARM_WRIST_MOTOR_INVER_OUTPUT = true;
    public static final boolean ARM_WRIST_MOTOR_INVERT_SENSOR = true;

    public static final boolean ARM_USE_SIMPLE_MODE = false;
    public static final boolean ARM_USE_MM = true;
    public static final boolean USE_IK_CONSTRAINTS = false;
    public static final boolean ARM_USE_GRAVITY_COMPENSATION = true;

    public static final double ARM_SHOULDER_PID_ADJUST_VEL = 5.0;
    public static final double ARM_WRIST_PID_ADJUST_VEL = 10.0;

    public static final double ARM_SHOULDER_MOTOR_PLAINPOSITIONAL_PID_KP = 0.01;
    public static final double ARM_SHOULDER_MOTOR_PLAINPOSITIONAL_PID_KI = 0.0;
    public static final double ARM_SHOULDER_MOTOR_PLAINPOSITIONAL_PID_KD = 0.0;
    public static final double ARM_SHOULDER_MOTOR_PLAINPOSITIONAL_PID_KF = 0.0;

    public static final double ARM_SHOULDER_PLAINPOSITIONAL_TMP_PID_KP = 0.06;
    public static final double ARM_SHOULDER_PLAINPOSITIONAL_TMP_PID_KI = 0.0;
    public static final double ARM_SHOULDER_PLAINPOSITIONAL_TMP_PID_KD = 0.03;
    public static final double ARM_SHOULDER_PLAINPOSITIONAL_TMP_PID_KF = 0.0;
    public static final double ARM_SHOULDER_TMP_PID_CRUISE_VELOC = 120.0;
    public static final double ARM_SHOULDER_TMP_PID_ACCEL = 100.0;

    public static final double ARM_WRIST_MOTOR_PID_KP = 0.08;
    public static final double ARM_WRIST_MOTOR_PID_KI = 0.0;
    public static final double ARM_WRIST_MOTOR_PID_KD = 0.0;
    public static final double ARM_WRIST_MOTOR_PID_KF = 0.0;

    public static final double ARM_WRIST_POSITION_MM_PID_KP = 0.08;
    public static final double ARM_WRIST_POSITION_MM_PID_KI = 0.0;
    public static final double ARM_WRIST_POSITION_MM_PID_KD = 0.0;
    public static final double ARM_WRIST_POSITION_MM_PID_KF = 0.0176; // 1023 / 58000
    public static final double ARM_WRIST_POSITION_MM_CRUISE_VELOCITY = 36864.0;
    public static final double ARM_WRIST_POSITION_MM_ACCELERATION = 36864.0;

    public static final boolean ARM_STALL_PROTECTION_ENABLED = false;
    public static final double ARM_SHOULDER_STALLED_CURRENT_THRESHOLD = 3.5;
    public static final double BATTERY_AVERAGE_EXPECTED_VOLTAGE = 12.0;
    public static final double ARM_SHOULDER_STALLED_POWER_THRESHOLD = TuningConstants.ARM_SHOULDER_STALLED_CURRENT_THRESHOLD * TuningConstants.BATTERY_AVERAGE_EXPECTED_VOLTAGE;
    public static final double ARM_SHOULDER_STALLED_VELOCITY_THRESHOLD = 8.0;
    public static final double ARM_WRIST_STALLED_CURRENT_THRESHOLD = 1.5;
    public static final double ARM_WRIST_STALLED_POWER_THRESHOLD = TuningConstants.ARM_WRIST_STALLED_CURRENT_THRESHOLD * TuningConstants.BATTERY_AVERAGE_EXPECTED_VOLTAGE;
    public static final double ARM_WRIST_STALLED_VELOCITY_THRESHOLD = 4.0;

    public static final double ARM_SHOULDER_POWER_TRACKING_DURATION = 0.250;
    public static final double ARM_SHOULDER_POWER_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND;
    public static final double ARM_SHOULDER_VELOCITY_TRACKING_DURATION = TuningConstants.ARM_SHOULDER_POWER_TRACKING_DURATION;
    public static final double ARM_SHOULDER_VELOCITY_SAMPLES_PER_SECOND = TuningConstants.ARM_SHOULDER_POWER_SAMPLES_PER_SECOND;

    public static final double ARM_WRIST_POWER_TRACKING_DURATION = 0.250;
    public static final double ARM_WRIST_POWER_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND;
    public static final double ARM_WRIST_VELOCITY_TRACKING_DURATION = TuningConstants.ARM_WRIST_POWER_TRACKING_DURATION;
    public static final double ARM_WRIST_VELOCITY_SAMPLES_PER_SECOND = TuningConstants.ARM_WRIST_POWER_SAMPLES_PER_SECOND;
}
