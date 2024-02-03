package frc.robot;

/**
 * All constants describing the physical structure of the robot (distances and sizes of things).
 * 
 * @author Will
 * 
 */
public class HardwareConstants
{
    public static final double MAX_ROBOT_HEIGHT = 78.0; // inches, max overall height
    public static final double MAX_ROBOT_EXTENSION = 48.0; // inches, max extension beyond frame perimeter

    //================================================== DriveTrain ==============================================================

    public static final boolean SDSDRIVETRAIN_STEER_MOTOR1_INVERT = true;
    public static final boolean SDSDRIVETRAIN_STEER_MOTOR2_INVERT = true;
    public static final boolean SDSDRIVETRAIN_STEER_MOTOR3_INVERT = true;
    public static final boolean SDSDRIVETRAIN_STEER_MOTOR4_INVERT = true;

    public static final boolean SDSDRIVETRAIN_DRIVE_MOTOR1_INVERT = false;
    public static final boolean SDSDRIVETRAIN_DRIVE_MOTOR2_INVERT = false;
    public static final boolean SDSDRIVETRAIN_DRIVE_MOTOR3_INVERT = false;
    public static final boolean SDSDRIVETRAIN_DRIVE_MOTOR4_INVERT = false;

    public static final double SDSDRIVETRAIN_STEER_GEAR_RATIO = 150.0 / 7.0; // According to SDS Mk4i code: (50.0 / 14.0) * (60.0 / 10.0) == ~21.43 : 1
    public static final double SDSDRIVETRAIN_STEER_DEGREES = 360.0;
    public static final double SDSDRIVETRAIN_STEER_TICK_DISTANCE = HardwareConstants.SDSDRIVETRAIN_STEER_DEGREES / HardwareConstants.SDSDRIVETRAIN_STEER_GEAR_RATIO; // in degrees
    public static final double SDSDRIVETRAIN_STEER_TICKS_PER_DEGREE = HardwareConstants.SDSDRIVETRAIN_STEER_GEAR_RATIO / HardwareConstants.SDSDRIVETRAIN_STEER_DEGREES; // in rotations

    public static final double SDSDRIVETRAIN_DRIVE_GEAR_RATIO = 36000.0 / 5880; // According to SDS Mk4i Very Fast code: (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0) == ~6.12 : 1
    public static final double SDSDRIVETRAIN_DRIVE_WHEEL_DIAMETER = 3.725; // SDS Mk4i code claims their 4-inch wheels are actually 3.95 inches now (in inches) We think its 3.95 - main 3.97
    public static final double SDSDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.SDSDRIVETRAIN_DRIVE_WHEEL_DIAMETER;
    public static final double SDSDRIVETRAIN_DRIVE_TICK_DISTANCE = HardwareConstants.SDSDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE / HardwareConstants.SDSDRIVETRAIN_DRIVE_GEAR_RATIO;
    public static final double SDSDRIVETRAIN_DRIVE_TICKS_PER_INCH = HardwareConstants.SDSDRIVETRAIN_DRIVE_GEAR_RATIO / HardwareConstants.SDSDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE;
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND = HardwareConstants.SDSDRIVETRAIN_DRIVE_TICK_DISTANCE; // converts rotations/sec into inches per second.
    public static final double SDSDRIVETRAIN_DRIVE_INCHES_PER_SECOND_TO_MOTOR_VELOCITY = HardwareConstants.SDSDRIVETRAIN_DRIVE_TICKS_PER_INCH; // converts inches per second into rotations/sec

    public static final double SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE = 22.75; // (in inches) 35" side-to-side with bumpers
    public static final double SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE = 22.75; // (in inches) 38" front-to-back with bumpers
    public static final double SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE = HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)
    public static final double SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE = HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)

    //================================================== DriveTrainNeo ==============================================================

    public static final boolean REVDRIVETRAIN_STEER_MOTOR1_INVERT_OUTPUT = false;
    public static final boolean REVDRIVETRAIN_STEER_MOTOR2_INVERT_OUTPUT = false;
    public static final boolean REVDRIVETRAIN_STEER_MOTOR3_INVERT_OUTPUT = false;
    public static final boolean REVDRIVETRAIN_STEER_MOTOR4_INVERT_OUTPUT = false;

    public static final boolean REVDRIVETRAIN_STEER_MOTOR1_INVERT_SENSOR = true;
    public static final boolean REVDRIVETRAIN_STEER_MOTOR2_INVERT_SENSOR = true;
    public static final boolean REVDRIVETRAIN_STEER_MOTOR3_INVERT_SENSOR = true;
    public static final boolean REVDRIVETRAIN_STEER_MOTOR4_INVERT_SENSOR = true;

    public static final boolean REVDRIVETRAIN_DRIVE_MOTOR1_INVERT_OUTPUT = false;
    public static final boolean REVDRIVETRAIN_DRIVE_MOTOR2_INVERT_OUTPUT = false;
    public static final boolean REVDRIVETRAIN_DRIVE_MOTOR3_INVERT_OUTPUT = false;
    public static final boolean REVDRIVETRAIN_DRIVE_MOTOR4_INVERT_OUTPUT = false;

    public static final boolean REVDRIVETRAIN_DRIVE_MOTOR1_INVERT_SENSOR = false;
    public static final boolean REVDRIVETRAIN_DRIVE_MOTOR2_INVERT_SENSOR = false;
    public static final boolean REVDRIVETRAIN_DRIVE_MOTOR3_INVERT_SENSOR = false;
    public static final boolean REVDRIVETRAIN_DRIVE_MOTOR4_INVERT_SENSOR = false;

    public static final double REVDRIVETRAIN_STEER_TICKS_PER_REVOLUTION = 1.0;
    public static final double REVDRIVETRAIN_STEER_GEAR_RATIO = 1.0; // Rev throughbore encoder is connected to the output shaft of the swerve module
    public static final double REVDRIVETRAIN_STEER_DEGREES = 360.0;
    public static final double REVDRIVETRAIN_STEER_TICK_DISTANCE = HardwareConstants.REVDRIVETRAIN_STEER_DEGREES / (HardwareConstants.REVDRIVETRAIN_STEER_GEAR_RATIO * HardwareConstants.REVDRIVETRAIN_STEER_TICKS_PER_REVOLUTION); // in degrees
    public static final double REVDRIVETRAIN_STEER_TICKS_PER_DEGREE = (HardwareConstants.REVDRIVETRAIN_STEER_GEAR_RATIO * HardwareConstants.REVDRIVETRAIN_STEER_TICKS_PER_REVOLUTION) / HardwareConstants.REVDRIVETRAIN_STEER_DEGREES; // in ticks
    public static final double REVDRIVETRAIN_STEER_MOTOR_VELOCITY_TO_DEGREES_PER_SECOND = HardwareConstants.REVDRIVETRAIN_DRIVE_TICK_DISTANCE / 60.0; // converts RPM into degrees per second.

    public static final double REVDRIVETRAIN_DRIVE_TICKS_PER_REVOLUTION = 1.0;
    public static final double REVDRIVETRAIN_DRIVE_GEAR_RATIO = (45.0 * 22.0) / (14.0 * 15.0); // Gear ratios according to Rev (14 may instead be 12 or 13 depending on the chosen pinion gear)
    public static final double REVDRIVETRAIN_DRIVE_WHEEL_DIAMETER = 3.0; // Wheels are approximately this diameter
    public static final double REVDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.REVDRIVETRAIN_DRIVE_WHEEL_DIAMETER;
    public static final double REVDRIVETRAIN_DRIVE_TICK_DISTANCE = HardwareConstants.REVDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE / (HardwareConstants.REVDRIVETRAIN_DRIVE_GEAR_RATIO * HardwareConstants.REVDRIVETRAIN_DRIVE_TICKS_PER_REVOLUTION);
    public static final double REVDRIVETRAIN_DRIVE_TICKS_PER_INCH = (HardwareConstants.REVDRIVETRAIN_DRIVE_GEAR_RATIO * HardwareConstants.REVDRIVETRAIN_DRIVE_TICKS_PER_REVOLUTION) / HardwareConstants.REVDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE;
    public static final double REVDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND = HardwareConstants.REVDRIVETRAIN_DRIVE_TICK_DISTANCE / 60.0; // converts RPM into inches per second.

    public static final double REVDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE = 22.75; // (in inches) 35" side-to-side with bumpers
    public static final double REVDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE = 22.75; // (in inches) 38" front-to-back with bumpers
    public static final double REVDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE = HardwareConstants.REVDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)
    public static final double REVDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE = HardwareConstants.REVDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)

    //=============================================== Arm =====================================================================//

    // NOTE: "tick distance" is how far one tick is in degrees, multiply by this to get angle in degrees from ticks.
    public static final double ARM_SHOULDER_TICK_DISTANCE = 1.0;
    public static final double ARM_SHOULDER_TICKS_PER_DEGREE = 1.0;
    public static final double ARM_WRIST_TICK_DISTANCE = 1.0;
    public static final double ARM_WRIST_TICKS_PER_DEGREE = 1.0;

    public static final double ARM_HUMERUS_LENGTH = 36.0; // length of arm section between shoulder joint and wrist joint ("humerus")
    public static final double ARM_ULNA_LENGTH = 18.0; // length of (end effector) arm section between wrist joint and shooter ("ulna")
    public static final double ARM_SHOOTER_RETRACT_X_POS = 100; 
    public static final double ARM_SHOOTER_RETRACT_Z_POS = 100;

    //=============================================== Vision ==================================================================//

    public static final double CAMERA_TO_ARM_X_OFFSET = 20;
    public static final double CAMERA_TO_ARM_Z_OFFSET = 20;

    //=============================================== EndEffector =====================================================================//
    //TODO get acutal offsets

    public static final double END_EFFECTOR_PIVOT_LENGTH = 18; // inches
    public static final double END_EFFECTOR_PIVOT_AXIS_OFFSET = 4; //inches

    public static final double SHOOTER_NEAR_FLYWHEEL_TICK_DISTANCE = 1.0;
    public static final double SHOOTER_FAR_FLYWHEEL_TICK_DISTANCE = 1.0;
}
