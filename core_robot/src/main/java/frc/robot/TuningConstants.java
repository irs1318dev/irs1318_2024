package frc.robot;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * All constants related to tuning the operation of the robot.
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = true;
    public static boolean THROW_EXCEPTIONS = false;
    public static boolean LOG_EXCEPTIONS = true;
    public static double LOOP_DURATION = 0.02; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)
    public static int LOOPS_PER_SECOND = 50; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)

    public static final boolean EXPECT_UNUSED_JOYSTICKS = true;

    //================================================== Magic Values ==============================================================

    public static final double MAGIC_NULL_VALUE = -1318.0;
    public static final double ZERO = 0.0;
    public static final double EUGENE_HAROLD_KRABS = 0.0;
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

    public static final double AMP_OT = 90.0;
    public static final double SOURCE_OT = 290;

    public static final double DRIVE_P19_SHOOT_ORIENTATION = 225;
    public static final double DRIVE_P17_SHOOT_ORIENTATION = 135;

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

    public static final List<Integer> VISION_SPEAKER_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_SPEAKER_CENTER_ID);
    public static final String VISION_SPEAKER_BLUE_STRING = TuningConstants.VISION_SPEAKER_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_SPEAKER_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_SPEAKER_CENTER_ID);
    public static final String VISION_SPEAKER_RED_STRING = TuningConstants.VISION_SPEAKER_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    public static final List<Integer> VISION_STAGE_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_STAGE_LEFT_ID, TuningConstants.APRILTAG_BLUE_CENTER_STAGE_ID, TuningConstants.APRILTAG_BLUE_STAGE_RIGHT_ID);
    public static final String VISION_STAGE_BLUE_STRING = TuningConstants.VISION_STAGE_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_STAGE_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_STAGE_LEFT_ID, TuningConstants.APRILTAG_RED_CENTER_STAGE_ID, TuningConstants.APRILTAG_RED_STAGE_RIGHT_ID);
    public static final String VISION_STAGE_RED_STRING = TuningConstants.VISION_STAGE_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    public static final List<Integer> VISION_AMP_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_AMP_ID);
    public static final String VISION_AMP_BLUE_STRING = TuningConstants.VISION_AMP_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_AMP_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_AMP_ID);
    public static final String VISION_AMP_RED_STRING = TuningConstants.VISION_AMP_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Finding AprilTags to determine if theres enough valid data to translate 
    public static final int TAGS_MISSED_THRESHOLD = 30;
    public static final int TAGS_FOUND_THRESHOLD = 5;
    public static final double ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE = 1.0; // in inches

    // Acceptable vision centering range values in degrees
    public static final double MAX_PID_TURNING_RANGE_DEGREES = 2.0;

    // How long the robot system must remain centered on the target when using time
    public static final double PID_TURNING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;
    public static final double MAX_VISION_ACCEPTABLE_STRAFE_DISTANCE = 0.7;

    // Acceptable vision distance from tape in angles 
    public static final double MAX_VISION_ACCEPTABLE_MOVING_RR_ANGLE_ERROR = 4.0;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double STATIONARY_PID_TURNING_PID_KP = 0.02;
    public static final double STATIONARY_PID_TURNING_PID_KI = 0.0;
    public static final double STATIONARY_PID_TURNING_PID_KD = 0.0;
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

    //================================================== SDS DriveTrain ==============================================================

    public static final boolean SDSDRIVETRAIN_STEER_MOTORS_USE_MOTION_MAGIC = true;
    public static final boolean SDSDRIVETRAIN_USE_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION = false;
    public static final double SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP = 0.02;

    public static final boolean SDSDRIVETRAIN_USE_ODOMETRY = true;
    public static final boolean SDSDRIVETRAIN_RESET_ON_ROBOT_START = true;
    public static final boolean SDSDRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START = true;
    public static final boolean SDSDRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START = true;

    public static final double SDSDRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? 0.3046875 : 0.17212; // -0.198486 + 0.5; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? 0.3129882 : 0.15698;//-0.186768 + 0.5; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? 0.4375 : -0.28979;//0.46337890625; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? -0.1572265 : -0.35327;//0.336670 + 0.5; // rotations

    public static final boolean SDSDRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING = true;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -4.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE = 4.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final boolean SDSDRIVETRAIN_USE_ROTATIONAL_RATE_LIMITING = true;
    public static final double SDSDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -4.0 * TuningConstants.SDSDRIVETRAIN_TURN_SCALE;
    public static final double SDSDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_POSITIVE_RATE = 4.0 * TuningConstants.SDSDRIVETRAIN_TURN_SCALE;

    // Position PID (angle) per-module
    public static final double SDSDRIVETRAIN_STEER_MOTOR_POSITION_PID_KS = HardwareConstants.SDSDRIVETRAIN_STEER_TICKS_PER_DEGREE;

    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KP : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KP;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KI : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KI;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KD : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KD;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KF : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KF;

    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KP : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KP;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KI : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KI;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KD : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KD;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KV = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KV : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KV;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KS = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KS : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KS;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_CRUISE_VELOC = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_CRUISE_VELOC : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_CRUISE_VELOC;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_ACCEL = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_ACCEL : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_ACCEL;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_JERK = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_JERK : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_JERK;

    // STEER PRACTICE
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KP = 1.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);

    // STEER RPM ~107.0 was highest speed at full throttle FF on blocks
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KP = 0.3333333 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KV = 0.00934579 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KS = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_CRUISE_VELOC = 100.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_ACCEL = 600.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_JERK = 9999.0;

    // STEER COMP
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KP = 1.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);

    // STEER RPM ~107.0 was highest speed at full throttle FF on blocks
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KP = 0.3333333 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KV = 0.00934579 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KS = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_CRUISE_VELOC = 100.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_ACCEL = 600.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_JERK = 9999.0;

    // Velocity PID (drive) per-module
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_COMP_VELOCITY_PID_KS : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_PRACTICE_VELOCITY_PID_KS; // RPM ~110.0 was highest speed at full throttle FF on blocks

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KP : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KP;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KI : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KI;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KD : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KD;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KF : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KF;

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KP : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KP;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KI : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KI;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KD : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KD;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KF : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KF;

    // DRIVE PRACTICE
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_PRACTICE_VELOCITY_PID_KS = 88.0; // RPM ~110.0 was highest speed at full throttle FF on blocks

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KP = 0.02 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KF = 0.00909 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0); // 100% control authority (on blocks)

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KP = 2.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);

    // DRIVE COMP
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_COMP_VELOCITY_PID_KS = 88.0; // RPM ~104.0 was highest speed at full throttle FF on blocks

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KP = 0.02 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KF = 0.00961538 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0); // 100% control authority (on blocks)

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KP = 2.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);

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

    public static final boolean INTAKE_MOTOR_INVERT_OUTPUT = TuningConstants.COMPETITION_ROBOT ? true : true;
    public static final double EFFECTOR_INTAKE_IN_POWER = 0.6;
    public static final double EFFECTOR_INTAKE_OUT_POWER = -0.4;
    public static final double EFFECTOR_INTAKE_FEED_SHOOTER_POWER = 0.9;

    public static final boolean NEAR_SHOOTER_MOTOR_INVERT_SENSOR = false;
    public static final boolean NEAR_SHOOTER_MOTOR_INVERT_OUTPUT = false;

    public static final boolean FAR_SHOOTER_MOTOR_INVERT_SENSOR = false;
    public static final boolean FAR_SHOOTER_MOTOR_INVERT_OUTPUT = false;

    public static final double SHOOTER_NEAR_FLYWHEEL_MAX_VELOCITY = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SHOOTER_NEAR_FLYWHEEL_COMP_MAX_VELOCITY : TuningConstants.SHOOTER_NEAR_FLYWHEEL_PRACTICE_MAX_VELOCITY; // (RPM)
    public static final double SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SHOOTER_NEAR_FLYWHEEL_COMP_MOTOR_PID_KP : TuningConstants.SHOOTER_NEAR_FLYWHEEL_PRACTICE_MOTOR_PID_KP;
    public static final double SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SHOOTER_NEAR_FLYWHEEL_COMP_MOTOR_PID_KI : TuningConstants.SHOOTER_NEAR_FLYWHEEL_PRACTICE_MOTOR_PID_KI;
    public static final double SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SHOOTER_NEAR_FLYWHEEL_COMP_MOTOR_PID_KD : TuningConstants.SHOOTER_NEAR_FLYWHEEL_PRACTICE_MOTOR_PID_KD;
    public static final double SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SHOOTER_NEAR_FLYWHEEL_COMP_MOTOR_PID_KF : TuningConstants.SHOOTER_NEAR_FLYWHEEL_PRACTICE_MOTOR_PID_KF;

    public static final double SHOOTER_FAR_FLYWHEEL_MAX_VELOCITY = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SHOOTER_FAR_FLYWHEEL_COMP_MAX_VELOCITY : TuningConstants.SHOOTER_FAR_FLYWHEEL_PRACTICE_MAX_VELOCITY; // (RPM)
    public static final double SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SHOOTER_FAR_FLYWHEEL_COMP_MOTOR_PID_KP : TuningConstants.SHOOTER_FAR_FLYWHEEL_PRACTICE_MOTOR_PID_KP;
    public static final double SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SHOOTER_FAR_FLYWHEEL_COMP_MOTOR_PID_KI : TuningConstants.SHOOTER_FAR_FLYWHEEL_PRACTICE_MOTOR_PID_KI;
    public static final double SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SHOOTER_FAR_FLYWHEEL_COMP_MOTOR_PID_KD : TuningConstants.SHOOTER_FAR_FLYWHEEL_PRACTICE_MOTOR_PID_KD;
    public static final double SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SHOOTER_FAR_FLYWHEEL_COMP_MOTOR_PID_KF : TuningConstants.SHOOTER_FAR_FLYWHEEL_PRACTICE_MOTOR_PID_KF;

    // SHOOTER PRACTICE
    public static final double SHOOTER_NEAR_FLYWHEEL_PRACTICE_MAX_VELOCITY = 5300.0; // (RPM)
    public static final double SHOOTER_NEAR_FLYWHEEL_PRACTICE_MOTOR_PID_KP = 0.00040;
    public static final double SHOOTER_NEAR_FLYWHEEL_PRACTICE_MOTOR_PID_KI = 0.0;
    public static final double SHOOTER_NEAR_FLYWHEEL_PRACTICE_MOTOR_PID_KD = 0.015;
    public static final double SHOOTER_NEAR_FLYWHEEL_PRACTICE_MOTOR_PID_KF = 0.00018868;

    public static final double SHOOTER_FAR_FLYWHEEL_PRACTICE_MAX_VELOCITY = 5000.0; // (RPM)
    public static final double SHOOTER_FAR_FLYWHEEL_PRACTICE_MOTOR_PID_KP = 0.00032;
    public static final double SHOOTER_FAR_FLYWHEEL_PRACTICE_MOTOR_PID_KI = 0.0;
    public static final double SHOOTER_FAR_FLYWHEEL_PRACTICE_MOTOR_PID_KD = 0.015;
    public static final double SHOOTER_FAR_FLYWHEEL_PRACTICE_MOTOR_PID_KF = 0.0002;

    // SHOOTER COMP
    public static final double SHOOTER_NEAR_FLYWHEEL_COMP_MAX_VELOCITY = 5200.0; // (RPM)
    public static final double SHOOTER_NEAR_FLYWHEEL_COMP_MOTOR_PID_KP = 0.00040;
    public static final double SHOOTER_NEAR_FLYWHEEL_COMP_MOTOR_PID_KI = 0.0;
    public static final double SHOOTER_NEAR_FLYWHEEL_COMP_MOTOR_PID_KD = 0.015;
    public static final double SHOOTER_NEAR_FLYWHEEL_COMP_MOTOR_PID_KF = 0.0001923;

    public static final double SHOOTER_FAR_FLYWHEEL_COMP_MAX_VELOCITY = 5200.0; // (RPM)
    public static final double SHOOTER_FAR_FLYWHEEL_COMP_MOTOR_PID_KP = 0.00040;
    public static final double SHOOTER_FAR_FLYWHEEL_COMP_MOTOR_PID_KI = 0.0;
    public static final double SHOOTER_FAR_FLYWHEEL_COMP_MOTOR_PID_KD = 0.015;
    public static final double SHOOTER_FAR_FLYWHEEL_COMP_MOTOR_PID_KF = 0.0001923;

    public static final int FLYWHEEL_STALL_LIMIT = 100;
    public static final int FLYWHEEL_FREE_LIMIT = 100;
    public static final int FLYWHEEL_RPM_LIMIT = 2000;
    public static final int FLYWHEEL_SENSOR_FRAME_PERIOD_MS = 10;

    public static final double INTAKE_THROUGHBEAM_CUTOFF = 2.7;
    public static final double EFFECTOR_OUTTAKE_DURATION = 0.5; // time from when ring no longer between through beams to out of the effector when outtaking
    public static final double EFFECTOR_SHOOTING_DURATION = 1.0;

    public static final double FLYWHEEL_ALLOWABLE_ERROR_RANGE = 250;

    //==================================================== ShootNoteTask =====================================================

    // TODO get the actual values here
    public static final double GRAVITY_CONSTANT = 384; // inches per second per second

    public static final double APRILTAG_TARGET_OFFSET_X = 0; // inches
    public static final double APRILTAG_TARGET_OFFSET_Y = 48; // inches
    public static final double CAMERA_SHOOTER_PIVOT_OFFSET_X = 12; // inches
    public static final double CAMERA_SHOOTER_PIVOT_OFFSET_Y = 24; // inches

    public static final int ANGLE_FINDING_ITERATIONS = 20;

    public static final double SHOOTER_FINAL_ANGLE_OFFSET = 2.5; //degrees

    public static final double SHOOTER_MAX_VELOCITY = 730; //inches per second

    public static final double KICK_OUTTAKE_TIME = 0.2; //seconds
    public static final double KICK_INTAKE_TIME = 0.5; //seconds

    public static final double SHOOTER_DRAG_COMPENSATION_MULTIPLIER = 1.1; //multiplier

    //==================================================== Arm ==============================================================

    public static final double ARM_SHOULDER_POWER_STRENGTH = 0.5;
    public static final double ARM_WRIST_POWER_STRENGTH = 0.6;
    public static final double ARM_POWER_EXPONENTIAL = 1.0;
    public static final double ARM_SHOULDER_DEAD_ZONE = 0.1;
    public static final double ARM_WRIST_DEAD_ZONE = 0.1;

    public static final double ARM_SHOULDER_MIN_POSITION = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_COMP_MIN_POSITION : TuningConstants.ARM_SHOULDER_PRACTICE_MIN_POSITION; // in degrees
    public static final double ARM_SHOULDER_MAX_POSITION = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_COMP_MAX_POSITION : TuningConstants.ARM_SHOULDER_PRACTICE_MAX_POSITION; // in degrees
    public static final double ARM_WRIST_MIN_POSITION = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_COMP_MIN_POSITION : TuningConstants.ARM_WRIST_PRACTICE_MIN_POSITION; // in degrees
    public static final double ARM_WRIST_MAX_POSITION = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_COMP_MAX_POSITION : TuningConstants.ARM_WRIST_PRACTICE_MAX_POSITION; // in degrees

    public static final double ARM_SHOULDER_PRACTICE_MIN_POSITION = -28.1; // in degrees
    public static final double ARM_SHOULDER_PRACTICE_MAX_POSITION = 55.0; // in degrees
    public static final double ARM_WRIST_PRACTICE_MIN_POSITION = -112.0; // in degrees
    public static final double ARM_WRIST_PRACTICE_MAX_POSITION = 180.0; // in degrees

    public static final double ARM_SHOULDER_COMP_MIN_POSITION = -31.6; // in degrees
    public static final double ARM_SHOULDER_COMP_MAX_POSITION = 58.0; // in degrees
    public static final double ARM_WRIST_COMP_MIN_POSITION = -114.23; // in degrees
    public static final double ARM_WRIST_COMP_MAX_POSITION = 210.0; // in degrees

    // -------------------> SHOULDER POSITIONS <------------------- (all in degrees)
    public static final double ARM_SHOULDER_UNIVERSAL_DELTA = 3.0;

    public static final double ARM_SHOULDER_POSITION_STARTING_CONFIGURATION = TuningConstants.ARM_SHOULDER_MIN_POSITION;
    public static final double ARM_WRIST_POSITION_STARTING_CONFIGURATION = TuningConstants.ARM_WRIST_MIN_POSITION;

    public static final double ARM_SHOULDER_POSITION_LOWER_UNIVERSAL = TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION;
    public static final double ARM_WRIST_POSITION_LOWER_UNIVERSAL_MIN = TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION;
    public static final double ARM_WRIST_POSITION_LOWER_UNIVERSAL_MAX = TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP;
    public static final double ARM_WRIST_POSITION_STOWED = TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION;

    public static final double ARM_WRIST_POSITION_GROUND_PICKUP = TuningConstants.COMPETITION_ROBOT ? 35.37056350708008 : 37.0;
    public static final double ARM_WRIST_POSITION_GROUND_SHOT = TuningConstants.COMPETITION_ROBOT ? 22.095434188842773 : 28.634967803955078;// change ti p4

    public static final double ARM_WRIST_AUTO_P4_SHOT = 22;
    public static final double ARM_WRIST_AUTO_P2_SHOT = 25;
    public static final double ARM_WRIST_AUTO_P6_SHOT = 2.727619171142578;
    public static final double ARM_WRIST_AUTO_P3_SHOT = 16.201045989990234;
    public static final double ARM_WRIST_AUTO_P5_SHOT = 16.201045989990234;
    public static final double ARM_WRIST_AUTO_P7_SHOT = 16.201045989990234;

    //Postion for going slightly out before a arm reset task, to gain momentum past limit switches
    public static final double ARM_WRIST_POSITION_MINOR_TILT = 10 + ARM_WRIST_POSITION_STARTING_CONFIGURATION;

    // Position for a node where the wrist is being tucked under the shoulder in a dangerous way
    // we will probably never attempt to go to this node, but if the wrist/shoulder stalls at an
    // inopportune time, it may fall to it.
    public static final double ARM_SHOULDER_POSITION_TUCKED_UNDER_TRANSIT = 0.0;
    public static final double ARM_WRIST_POSITION_TUCKED_UNDER_TRANSIT = 115.0;

    // a transit node for going between tucked positions and ground pickup,
    // or ground pickup to tucked positions
    public static final double ARM_SHOULDER_POSITION_TUCKED_TRANSIT = 0.0;
    public static final double ARM_WRIST_POSITION_TUCKED_TRANSIT = 82.0;

    // another transit node for going between tucked positions and ground pickup,
    // or ground pickup to tucked positions
    public static final double ARM_SHOULDER_POSITION_TUCKED_GROUND_TRANSIT = -20.0;
    public static final double ARM_WRIST_POSITION_TUCKED_GROUND_TRANSIT = 42.0;

    public static final double ARM_SHOULDER_POSITION_TUCKED = 10.236105918884277;
    public static final double ARM_WRIST_POSITION_TUCKED_SHOT = TuningConstants.COMPETITION_ROBOT ? 190.47601318359375 : 179.59384155273438;

    public static final double ARM_SHOULDER_POSITION_SOURCE_PICKUP = TuningConstants.COMPETITION_ROBOT ? 2.977274894714355 : TuningConstants.ARM_SHOULDER_POSITION_TUCKED;
    public static final double ARM_WRIST_POSITION_SOURCE_PICKUP = TuningConstants.COMPETITION_ROBOT ? -60.00739669799805 : -65.0;

    public static final double ARM_SHOULDER_POSITION_UPPER_UNIVERSAL = TuningConstants.COMPETITION_ROBOT ? 27.17892837524414 : 26;
    public static final double ARM_WRIST_POSITION_UPPER_UNIVERSAL_MIN = TuningConstants.ARM_WRIST_MIN_POSITION;
    public static final double ARM_WRIST_POSITION_UPPER_UNIVERSAL_MAX = TuningConstants.ARM_WRIST_MAX_POSITION;
    public static final double ARM_WRIST_POSITION_UPPER_UNIVERSAL_SHOT = TuningConstants.COMPETITION_ROBOT ? 63.87522888183594 : 71.33268737792969;

    public static final double ARM_SHOULDER_POSITION_AMP_SCORE = TuningConstants.COMPETITION_ROBOT ? 42.977256774902344 : 45.0;
    public static final double ARM_WRIST_POSITION_AMP_SCORE = TuningConstants.COMPETITION_ROBOT ? 100.67892456054688 : 100.0;

    public static final double ARM_SHOULDER_POSITION_INTAKE_FLIPPED = TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL;
    public static final double ARM_WRIST_POSITION_INTAKE_FLIPPED = -90.0;

    public static final double ARM_SHOULDER_POSITION_TRAP_INTERMEDIATE = 50.0;
    public static final double ARM_WRIST_POSITION_TRAP_INTERMEDIATE = -110.0;

    public static final double ARM_SHOULDER_POSITION_INTAKE_OBTUSE = TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL;
    public static final double ARM_WRIST_POSITION_INTAKE_OBTUSE = 113.0;

    public static final double ARM_WRIST_POSITION_GROUND_PICKUP_IK = TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP + 3.0;

    public static final double ARM_SHOULDER_POSITION_QUICK_TUCK = TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL;
    public static final double ARM_WRIST_POSITION_QUICK_TUCK = -66.80892944335938;

    public static final double ARM_SHOULDER_POSITION_AMP_OUTTAKE = -15.325244;
    public static final double ARM_WRIST_POSITION_AMP_OUTTAKE = -70.380218;
    

    public static final double ARM_SHOULDER_TRAP_OUTTAKE_POS = -1318;
    public static final double ARM_WRIST_TRAP_OUTTAKE_POS = -1318;

    public static final double ARM_WRIST_NODE_THRESHOLD = 3.0;
    public static final double ARM_SHOULDER_NODE_THRESHOLD = 3.0;
    public static final double ARM_WRIST_GOAL_THRESHOLD = 3.0;
    public static final double ARM_SHOULDER_GOAL_THRESHOLD = 6.0;

    public static final double ARM_SHOULDER_TRAP_SHOOT = 0 ;
    public static final double ARM_WRIST_TRAP_SHOOT = 0;

    // --------------------------------------> ARM GRAPH WEIGHTS <---------------------------------------
    // Shoulder Univ's
    public static final double STARTUP_AND_GROUND_PICKUP_WEIGHT = 1.1;
    public static final double STARTUP_AND_GROUND_SHOT_WEIGHT = 1.0;
    public static final double GROUND_PICKUP_AND_GROUND_SHOT_WEIGHT = 0.2;

    // Universal Transit's
    public static final double LOWER_UNIVERSAL_TRANSIT_WEIGHT = 0.7;
    public static final double UPPER_UNIVERSAL_TRANSIT_WEIGHT = 1.5;

    // Lower quartile stuff
    public static final double STARTUP_AND_SOURCE_PICKUP_WEIGHT = 0.5;
    public static final double STARTUP_AMP_OUTTAKE_WEIGHT = 0.6;
    public static final double STARTUP_AND_UPPER_INTAKE_FLIPPED_WEIGHT = 0.9;

    // Trap Int's
    public static final double UPPER_INTAKE_FLIPPED_AND_TRAP_INTER_WEIGHT = 0.4;
    public static final double STARTUP_AND_TRAP_INTERMEDIATE_WEIGHT = 1.2;

    // Upper Univ's
    public static final double UPPER_INTAKE_FLIPPED_AND_UPPER_OBTUSE_WEIGHT = 1.2;
    public static final double UPPER_INTAKE_FLIPPED_AND_UPPER_UNIV_WEIGHT = 1.0;
    public static final double UPPER_UNIV_AND_OBTUSE_WRIST_WEIGHT = 0.3;

    // Tucked Transit's
    public static final double UPPER_UNIV_AND_TUCKED_TRANSIT_WEIGHT = 0.6;
    public static final double TUCKED_TRANSIT_AND_TUCKED_WEIGHT = 0.7; // might be illegal / break robot
    public static final double UPPER_OBTUSE_WRIST_AND_TUCKED_TRANSIT_WEIGHT = 0.6;
    public static final double TUCKED_UNDER_TO_TUCKED_TRANSIT_WEIGHT = 0.2;
    public static final double AMP_SCORE_AND_TUCKED_TRANSIT_WEIGHT = 0.8;

    // Upper quartile stuff
    public static final double AMP_SCORE_AND_UPPER_UNIV_SHOT_WEIGHT = 0.4;
    public static final double AMP_SCORE_AND_OBTUSE_WRIST_WEIGHT = 0.3;

    // Tucked Ground Transit's
    public static final double GROUND_PICKUP_AND_TUCKED_GROUND_TRANSIT_WEIGHT = 0.2;
    public static final double GROUND_SHOT_AND_TUCKED_GROUND_TRANSIT_WEIGHT = 0.35;
    public static final double TUCKED_TRANSIT_AND_TUCKED_GROUND_TRANSIT_WEIGHT = 0.4;
    public static final double AMP_SCORE_AND_TUCKED_GROUND_TRANSIT_WEIGHT = 0.9; // might be illegal
    
    // Tucked
    public static final double TUCKED_TRANSIT_TO_TUCKED_WEIGHT = 0.6; // good
    public static final double UPPER_OBTEUSE_WRIST_AND_TUCKED_WEIGHT = 0.5;

    // -------------------------------------------> END OF WEIGHTS <---------------------------------------------

    public static final double ARM_SHOULDER_WEIGHT_MULTIPLIER = 2.0;
    public static final double ARM_WRIST_WEIGHT_MULTIPLIER = 1.0;

    // ----------------> OTHER THINGS <----------------

    public static final boolean ARM_SHOULDER_MOTOR_INVERT_OUTPUT = false;
    public static final boolean ARM_SHOULDER_MOTOR_FOLLOWER_INVERT_OUTPUT = false;
    public static final boolean ARM_WRIST_MOTOR_INVERT_OUTPUT = true;

    public static final boolean ARM_USE_SIMPLE_MODE = false;
    public static final boolean ARM_USE_MM = true;
    public static final boolean ARM_USE_IK_CONSTRAINTS = true;
    public static final boolean ARM_USE_GRAVITY_COMPENSATION = true;
    public static final boolean ARM_USE_COAST_ON_DISABLE = true;
    public static final boolean ARM_RESET_WRIST_WHEN_LIMIT_SWITCH_HIT = TuningConstants.COMPETITION_ROBOT ? true : false;
    public static final boolean ARM_USE_WRIST_ABSOLUTE_ENCODER_RESET = TuningConstants.COMPETITION_ROBOT ? true : false;

    public static final double ARM_SHOULDER_PID_ADJUST_VEL = 40.0;
    public static final double ARM_WRIST_PID_ADJUST_VEL = 20.0;

    public static final double ARM_SLOP_ADJUSTMENT_MULTIPLIER = 10.0;

    public static final double ARM_WRIST_RESET_STOPPED_VELOCITY_THRESHOLD = 0.1;
    public static final double ARM_WRIST_RESET_AT_POSITION_THRESHOLD = 3.0;
    public static final double ARM_WRIST_RESET_CORRECTION_THRESHOLD = 1.0;
    public static final double ARM_WRIST_ABSOLUTE_ENCODER_OFFSET = -0.443333;
    public static final double ARM_WRIST_RESET_DIFFERENCE_MAX = 45.0;

    // SHOULDER PID
    public static final double ARM_SHOULDER_MOTOR_POSITIONAL_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_POSITIONAL_PID_KP : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_PID_KP;
    public static final double ARM_SHOULDER_MOTOR_POSITIONAL_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_POSITIONAL_PID_KI : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_PID_KI;
    public static final double ARM_SHOULDER_MOTOR_POSITIONAL_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_POSITIONAL_PID_KD : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_PID_KD;
    public static final double ARM_SHOULDER_MOTOR_POSITIONAL_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_POSITIONAL_PID_KF : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_PID_KF;

    public static final double ARM_SHOULDER_MOTOR_POSITIONAL_TMP_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_POSITIONAL_TMP_PID_KP : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KP;
    public static final double ARM_SHOULDER_MOTOR_POSITIONAL_TMP_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_POSITIONAL_TMP_PID_KI : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KI;
    public static final double ARM_SHOULDER_MOTOR_POSITIONAL_TMP_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_POSITIONAL_TMP_PID_KD : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KD;
    public static final double ARM_SHOULDER_MOTOR_POSITIONAL_TMP_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_POSITIONAL_TMP_PID_KF : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KF;

    public static final double ARM_SHOULDER_MOTOR_TMP_PID_CRUISE_VELOC = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_TMP_PID_CRUISE_VELOC : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_TMP_PID_CRUISE_VELOC;
    public static final double ARM_SHOULDER_MOTOR_TMP_PID_ACCEL = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_TMP_PID_ACCEL : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_TMP_PID_ACCEL;

    // PRACTICE ROBOT SHOULDER PID
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_PID_KP = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_PID_KP : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_PID_KP;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_PID_KI = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_PID_KI : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_PID_KI;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_PID_KD = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_PID_KD : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_PID_KD;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_PID_KF = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_PID_KF : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_PID_KF;

    public static final double ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KP = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_TMP_PID_KP : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_TMP_PID_KP;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KI = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_TMP_PID_KI : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_TMP_PID_KI;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KD = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_TMP_PID_KD : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_TMP_PID_KD;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KF = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_TMP_PID_KF : TuningConstants.ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_TMP_PID_KF;

    public static final double ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_PID_KP = 0.01;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_PID_KI = 0.0;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_PID_KD = 0.0;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_PID_KF = 0.0;

    public static final double ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_TMP_PID_KP = 0.06;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_TMP_PID_KI = 0.0;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_TMP_PID_KD = 0.03;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_PLAINPOSITIONAL_TMP_PID_KF = 0.0;

    public static final double ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_PID_KP = 0.01;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_PID_KI = 0.0;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_PID_KD = 0.0;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_PID_KF = 0.0;

    public static final double ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_TMP_PID_KP = 0.012;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_TMP_PID_KI = 0.0;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_TMP_PID_KD = 0.03;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_GRAVPOSITIONAL_TMP_PID_KF = 0.0;

    public static final double ARM_SHOULDER_MOTOR_PRACTICE_TMP_PID_CRUISE_VELOC = 120.0;
    public static final double ARM_SHOULDER_MOTOR_PRACTICE_TMP_PID_ACCEL = 200.0;

    // COMP ROBOT SHOULDER PID
    public static final double ARM_SHOULDER_MOTOR_COMP_POSITIONAL_PID_KP = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_PID_KP : TuningConstants.ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_PID_KP;
    public static final double ARM_SHOULDER_MOTOR_COMP_POSITIONAL_PID_KI = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_PID_KI : TuningConstants.ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_PID_KI;
    public static final double ARM_SHOULDER_MOTOR_COMP_POSITIONAL_PID_KD = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_PID_KD : TuningConstants.ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_PID_KD;
    public static final double ARM_SHOULDER_MOTOR_COMP_POSITIONAL_PID_KF = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_PID_KF : TuningConstants.ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_PID_KF;

    public static final double ARM_SHOULDER_MOTOR_COMP_POSITIONAL_TMP_PID_KP = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_TMP_PID_KP : TuningConstants.ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_TMP_PID_KP;
    public static final double ARM_SHOULDER_MOTOR_COMP_POSITIONAL_TMP_PID_KI = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_TMP_PID_KI : TuningConstants.ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_TMP_PID_KI;
    public static final double ARM_SHOULDER_MOTOR_COMP_POSITIONAL_TMP_PID_KD = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_TMP_PID_KD : TuningConstants.ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_TMP_PID_KD;
    public static final double ARM_SHOULDER_MOTOR_COMP_POSITIONAL_TMP_PID_KF = TuningConstants.ARM_USE_GRAVITY_COMPENSATION ? TuningConstants.ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_TMP_PID_KF : TuningConstants.ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_TMP_PID_KF;

    public static final double ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_PID_KP = 0.01;
    public static final double ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_PID_KI = 0.0;
    public static final double ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_PID_KD = 0.0;
    public static final double ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_PID_KF = 0.0;

    public static final double ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_TMP_PID_KP = 0.06;
    public static final double ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_TMP_PID_KI = 0.0;
    public static final double ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_TMP_PID_KD = 0.03;
    public static final double ARM_SHOULDER_MOTOR_COMP_PLAINPOSITIONAL_TMP_PID_KF = 0.0;

    public static final double ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_PID_KP = 0.01;
    public static final double ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_PID_KI = 0.0;
    public static final double ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_PID_KD = 0.0;
    public static final double ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_PID_KF = 0.0;

    public static final double ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_TMP_PID_KP = 0.08;
    public static final double ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_TMP_PID_KI = 0.0;
    public static final double ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_TMP_PID_KD = 0.03;
    public static final double ARM_SHOULDER_MOTOR_COMP_GRAVPOSITIONAL_TMP_PID_KF = 0.0;

    public static final double ARM_SHOULDER_MOTOR_COMP_TMP_PID_CRUISE_VELOC = 120.0;
    public static final double ARM_SHOULDER_MOTOR_COMP_TMP_PID_ACCEL = 200.0;

    // WRSIT PID
    public static final double ARM_WRIST_MOTOR_POSITIONAL_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_MOTOR_COMP_POSITIONAL_PID_KP : TuningConstants.ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_PID_KP;
    public static final double ARM_WRIST_MOTOR_POSITIONAL_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_MOTOR_COMP_POSITIONAL_PID_KI : TuningConstants.ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_PID_KI;
    public static final double ARM_WRIST_MOTOR_POSITIONAL_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_MOTOR_COMP_POSITIONAL_PID_KD : TuningConstants.ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_PID_KD;
    public static final double ARM_WRIST_MOTOR_POSITIONAL_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_MOTOR_COMP_POSITIONAL_PID_KF : TuningConstants.ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_PID_KF;

    public static final double ARM_WRIST_MOTOR_POSITIONAL_TMP_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_MOTOR_COMP_POSITIONAL_TMP_PID_KP : TuningConstants.ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KP;
    public static final double ARM_WRIST_MOTOR_POSITIONAL_TMP_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_MOTOR_COMP_POSITIONAL_TMP_PID_KI : TuningConstants.ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KI;
    public static final double ARM_WRIST_MOTOR_POSITIONAL_TMP_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_MOTOR_COMP_POSITIONAL_TMP_PID_KD : TuningConstants.ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KD;
    public static final double ARM_WRIST_MOTOR_POSITIONAL_TMP_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_MOTOR_COMP_POSITIONAL_TMP_PID_KF : TuningConstants.ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KF;

    public static final double ARM_WRIST_MOTOR_TMP_PID_CRUISE_VELOC = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_MOTOR_COMP_TMP_PID_CRUISE_VELOC : TuningConstants.ARM_WRIST_MOTOR_PRACTICE_TMP_PID_CRUISE_VELOC;
    public static final double ARM_WRIST_MOTOR_TMP_PID_ACCEL = TuningConstants.COMPETITION_ROBOT ? TuningConstants.ARM_WRIST_MOTOR_COMP_TMP_PID_ACCEL : TuningConstants.ARM_WRIST_MOTOR_PRACTICE_TMP_PID_ACCEL;

    // PRACTICE ROBOT WRIST PID
    public static final double ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_PID_KP = 0.02;
    public static final double ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_PID_KI = 0.0;
    public static final double ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_PID_KD = 0.0;
    public static final double ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_PID_KF = 0.0;

    public static final double ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KP = 0.016;
    public static final double ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KI = 0.0;
    public static final double ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KD = 0.0;
    public static final double ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KF = 0.0;
    public static final double ARM_WRIST_MOTOR_PRACTICE_POSITIONAL_TMP_PID_KVF = 0.0;

    public static final double ARM_WRIST_MOTOR_PRACTICE_TMP_PID_CRUISE_VELOC = 180.0;
    public static final double ARM_WRIST_MOTOR_PRACTICE_TMP_PID_ACCEL = 360.0;//180.0;

    // COMP ROBOT WRIST PID
    public static final double ARM_WRIST_MOTOR_COMP_POSITIONAL_PID_KP = 0.02;
    public static final double ARM_WRIST_MOTOR_COMP_POSITIONAL_PID_KI = 0.0;
    public static final double ARM_WRIST_MOTOR_COMP_POSITIONAL_PID_KD = 0.0;
    public static final double ARM_WRIST_MOTOR_COMP_POSITIONAL_PID_KF = 0.0;

    public static final double ARM_WRIST_MOTOR_COMP_POSITIONAL_TMP_PID_KP = 0.20;
    public static final double ARM_WRIST_MOTOR_COMP_POSITIONAL_TMP_PID_KI = 0.0;
    public static final double ARM_WRIST_MOTOR_COMP_POSITIONAL_TMP_PID_KD = 0.0;
    public static final double ARM_WRIST_MOTOR_COMP_POSITIONAL_TMP_PID_KF = 0.0;
    public static final double ARM_WRIST_MOTOR_COMP_POSITIONAL_TMP_PID_KVF = 0.0;

    public static final double ARM_WRIST_MOTOR_COMP_TMP_PID_CRUISE_VELOC = 240.0;
    public static final double ARM_WRIST_MOTOR_COMP_TMP_PID_ACCEL = 360.0;

    public static final boolean ARM_STALL_PROTECTION_ENABLED = true;
    public static final double ARM_SHOULDER_STALLED_CURRENT_BUFFER = 8.0;
    public static final double ARM_SHOULDER_STALLED_CURRENT_THRESHOLD = 8.0;
    public static final double BATTERY_AVERAGE_EXPECTED_VOLTAGE = 12.0;
    public static final double PERCENT_OUTPUT_MULTIPLIER = 40.0;
    public static final double ARM_SHOULDER_STALLED_POWER_THRESHOLD = TuningConstants.ARM_SHOULDER_STALLED_CURRENT_THRESHOLD * TuningConstants.BATTERY_AVERAGE_EXPECTED_VOLTAGE;
    public static final double ARM_SHOULDER_STALLED_VELOCITY_THRESHOLD = 5.0; // degrees per second
    public static final double ARM_WRIST_STALLED_CURRENT_THRESHOLD = 6.0;
    public static final double ARM_WRIST_STALLED_POWER_THRESHOLD = TuningConstants.ARM_WRIST_STALLED_CURRENT_THRESHOLD * TuningConstants.BATTERY_AVERAGE_EXPECTED_VOLTAGE;
    public static final double ARM_WRIST_STALLED_VELOCITY_THRESHOLD = 0.5; // degrees per second
    public static final double ARM_SHOULDER_MOTOR_POWER_DIFFERENCE = 0.50; // Percentage difference allowed between the two motors
    public static final double ARM_SHOULDER_MOTOR_POWER_MIN_DIFFERENCE = 12.0; // Minimum average power consumption to pay attention to the difference

    // Arm/shoulder zeroingT
    public static final double ARM_SHOULDER_ZEROING_VELOCITY_THRESHOLD = 1.0; // degrees per second
    public static final double ARM_WRIST_ZEROING_VELOCITY_POS_THRESHOLD = 0.5; // degrees per second
    public static final double ARM_WRIST_ZEROING_VELOCITY_POW_THRESHOLD = 5.0; // degrees per second
    public static final double ARM_WRIST_ZEROING_POSITION_THRESHOLD = 12.0;
    public static final double ARM_SHOULDER_ZEROING_POSITION_THRESHOLD = 3.0;
    public static final double ARM_WRIST_ZEROING_POWER = -0.2;
    public static final double ARM_SHOULDER_ZEROING_POWER = -0.05;

    public static final double ARM_SHOULDER_POWER_TRACKING_DURATION = 0.250;
    public static final double ARM_SHOULDER_POWER_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND;
    public static final double ARM_SHOULDER_VELOCITY_TRACKING_DURATION = TuningConstants.ARM_SHOULDER_POWER_TRACKING_DURATION;
    public static final double ARM_SHOULDER_VELOCITY_SAMPLES_PER_SECOND = TuningConstants.ARM_SHOULDER_POWER_SAMPLES_PER_SECOND;

    public static final double ARM_WRIST_POWER_TRACKING_DURATION = 0.250;
    public static final double ARM_WRIST_POWER_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND;
    public static final double ARM_WRIST_VELOCITY_TRACKING_DURATION = TuningConstants.ARM_WRIST_POWER_TRACKING_DURATION;
    public static final double ARM_WRIST_VELOCITY_SAMPLES_PER_SECOND = TuningConstants.ARM_WRIST_POWER_SAMPLES_PER_SECOND;

    public static final double[] ARM_GRAVITY_COMPENSATION_SHOULDER_SAMPLE_LOCATIONS =
        {
            TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, // shoulder first position
            -15.0, // shoulder second position
            0.0, // shoulder third position
            15.0, // shoulder fourth position
            30.0, // shoulder fifth position
            45.0 // shoulder sixth position
        };

    public static final double[] ARM_GRAVITY_COMPENSATION_WRIST_SAMPLE_LOCATIONS =
        {
            TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION, // wrist first position
            -90.0, // wrist second position
            -60.0, // wrist third position
            -30.0, // wrist fourth position
            0.0, // wrist fifth position
            30.0, // wrist sixth position
            60.0, // wrist seventh position
            90.0, // wrist eighth position
            120.0, // wrist ninth position
            150.0, // wrist tenth position
            180.0 // wrist eleventh position
        };

    // percent-output values to hold shoulder at the desired shoulder and wrist positions
    public static final double ARM_MAX_GRAVITY_COMPENSATION = 0.2;
    public static final double[][] ARM_GRAVITY_COMPENSATION_SAMPLES =
        {
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // shoulder starting position
            { 0.057802058756351, 0.049989320337772, 0.049989320337772, 0.057649463415146, 0.069978944957256, 0.077394939959049, 0.079989016056061, 0.0, 0.0, 0.0, 0.0 }, // shoulder second position
            { 0.042695395648479, 0.055574204772711, 0.049989320337772, 0.057649463415146, 0.074983976781368, 0.081989016056061, 0.086994047880173, 0.089999087154865, 0.089999087154865, 0.093813896179199, 0.0949736014008522}, // shoulder third position
            { 0.059999391436577, 0.049989320337772, 0.049989320337772, 0.049989320337772, 0.074983976781368, 0.079989016056061, 0.084994047880173, 0.089999087154865, 0.089999087154865, 0.094973601400852, 0.0949736014008522}, // shoulder fourth position
            { 0.049989320337772, 0.03662221133709, 0.049989320337772, 0.049989320337772, 0.049989320337772, 0.079989016056061, 0.084994047880173, 0.089999087154865, 0.089999087154865, 0.085604421794415, 0.0949736014008522}, // shoulder fifth position
            { 0.049989320337772, 0.049989320337772, 0.049989320337772, 0.070894494652748, 0.099978640675545, 0.119978640675545, 0.099978640675545, 0.099978640675545, 0.099978640675545, 0.099978640675545, 0.099978640675545} // shoulder sixth position
        };


    //==================================================== Climber ==============================================================

    public static final boolean CLIMBER_MOTOR_INVERT_OUTPUT = false;
    public static final boolean CLIMBER_MOTOR_FOLLOWER_INVERT_OUTPUT = true;
    
    public static final double CLIMBER_WINCH_DOWN_POWER = -0.85;
    //public static final double CLIMBER_WINCH_DOWN_POWER = -0.5;

    public static final double CLIMBER_SERVO_UP_POSITION = 0.0;
    public static final double CLIMBER_SERVO_DOWN_POSITION = 1.0;

    public static final double CLIMBER_FULL_EXTEND_TIME = -1318; // in seconds
    public static final double CLIMBER_FULL_RETRACT_TIME = -1318; // in seconds

    //=================================================== Lookup-Table Shooter ==================================================
 
    // Distances that shot samples were captured from
    public static final double[] SHOOT_VISION_SAMPLE_DISTANCES = { 
       76.644, 86.644, 76.644, 76.644, 86.644, 96.644, 106.644, 116.644, 126.644, 
       136.644, 146.644, 156.644, 166.644, 176.644, 186.644, 196.644, 206.644, 
       216.644, 226.644, 231.733 };

    // angles where successful shots were made at the given speed
    public static final double[] SHOOT_VISION_SAMPLE_ANGLES = { 50.0, 47.0, 45.0, 39.0, 37.0 }; // 20 angles

    public static final double SHOOT_VISION_SPEED = 4000;
    public static final int SHOOT_VISION_APRILTAG_NOT_FOUND_THRESHOLD = 20;
    public static final double SHOOT_VISION_WRIST_ACCURACY_THRESHOLD = 1.0;
}