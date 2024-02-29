package frc.robot;

import frc.lib.helpers.Helpers;

/**
 * All constants describing the physical structure of the robot (distances and sizes of things).
 * 
 * @author Will
 * 
 */
public class HardwareConstants
{
    public static final double MAX_ROBOT_HEIGHT = 45.5; // inches, max overall height
    public static final double MAX_ROBOT_EXTENSION = 9.75; // inches, max extension beyond frame perimeter
    public static final double ROBOT_FRAME_DIMENSION = 28.0; // frame perimeter / 4.0
    public static final double ROBOT_HALF_FRAME_PERIMETER = 17.0; // "half frame perimeter" / 4.0"

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
    public static final double SDSDRIVETRAIN_DRIVE_WHEEL_DIAMETER = 3.875; // SDS Mk4i code claims their 4-inch wheels are actually 3.95 inches now (in inches) We think its 3.95 - main 3.97
    public static final double SDSDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.SDSDRIVETRAIN_DRIVE_WHEEL_DIAMETER;
    public static final double SDSDRIVETRAIN_DRIVE_TICK_DISTANCE = HardwareConstants.SDSDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE / HardwareConstants.SDSDRIVETRAIN_DRIVE_GEAR_RATIO;
    public static final double SDSDRIVETRAIN_DRIVE_TICKS_PER_INCH = HardwareConstants.SDSDRIVETRAIN_DRIVE_GEAR_RATIO / HardwareConstants.SDSDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE;
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND = HardwareConstants.SDSDRIVETRAIN_DRIVE_TICK_DISTANCE; // converts rotations/sec into inches per second.
    public static final double SDSDRIVETRAIN_DRIVE_INCHES_PER_SECOND_TO_MOTOR_VELOCITY = HardwareConstants.SDSDRIVETRAIN_DRIVE_TICKS_PER_INCH; // converts inches per second into rotations/sec

    public static final double SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE = 22.75; // (in inches) 35" side-to-side with bumpers
    public static final double SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE = 22.75; // (in inches) 38" front-to-back with bumpers
    public static final double SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE = HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)
    public static final double SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE = HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)

    //=============================================== Arm =====================================================================//

    // NOTE: "tick distance" is how far one tick is in degrees, multiply by this to get angle in degrees from ticks.
    public static final double ARM_SHOULDER_ENCODER_COUNTS_PER_ROTATION = 1.0; // count per rotation of axle - integrated encoder
    public static final double ARM_SHOULDER_GEAR_RATIO = 12.75 * 4.0; // 51:1 --> 12.75:1 in toughbox, 4:1 between chain sprockets.
    public static final double ARM_SHOULDER_TICK_DISTANCE = 360.0 / (HardwareConstants.ARM_SHOULDER_GEAR_RATIO * HardwareConstants.ARM_SHOULDER_ENCODER_COUNTS_PER_ROTATION); // degrees per rotation
    public static final double ARM_SHOULDER_TICK_VELOCITY = HardwareConstants.ARM_SHOULDER_TICK_DISTANCE / 60.0; // convert rotations per minute to degrees per second
    public static final double ARM_SHOULDER_TICKS_PER_DEGREE = (HardwareConstants.ARM_SHOULDER_GEAR_RATIO * HardwareConstants.ARM_SHOULDER_ENCODER_COUNTS_PER_ROTATION) / 360.0; // rotations per degree
    public static final double ARM_WRIST_ENCODER_COUNTS_PER_ROTATION = 1.0; // count per rotation of axle - integrated encoder
    public static final double ARM_WRIST_GEAR_RATIO = TuningConstants.COMPETITION_ROBOT ? 81.0 * 2.0 : 81.0 * 2.0; // 162:1 --> 81:1 in gearbox, 2:1 between chain sprockets.
    public static final double ARM_WRIST_TICK_DISTANCE = 360.0 / (HardwareConstants.ARM_WRIST_GEAR_RATIO * HardwareConstants.ARM_WRIST_ENCODER_COUNTS_PER_ROTATION); // degrees per rotation 
    public static final double ARM_WRIST_TICK_VELOCITY = HardwareConstants.ARM_WRIST_TICK_DISTANCE / 60.0; // convert rotations per minute to degrees per second
    public static final double ARM_WRIST_TICKS_PER_DEGREE = (HardwareConstants.ARM_WRIST_GEAR_RATIO * HardwareConstants.ARM_WRIST_ENCODER_COUNTS_PER_ROTATION) / 360.0; // rotations per degree

    public static final double ARM_HUMERUS_LENGTH = 24.25; // length of arm section between shoulder joint and wrist joint ("humerus")
    public static final double ARM_ULNA_LENGTH = 18.0; // length of (end effector) arm section between wrist joint and shooter ("ulna")

    public static final double ARM_WRIST_TO_SHOOTER_X = 3.630;//4.25;
    public static final double ARM_WRIST_TO_SHOOTER_Z = 8.032;//8.25;
    public static final double ARM_WRIST_TO_INTAKE_X = 12.146;//13;
    public static final double ARM_WRIST_TO_INTAKE_Z = 5.661;//8.05;
    public static final double ARM_WRIST_TO_SHOOTER_EDGE = Math.sqrt( Math.pow(HardwareConstants.ARM_WRIST_TO_SHOOTER_X, 2) +  Math.pow(HardwareConstants.ARM_WRIST_TO_SHOOTER_Z, 2));
    public static final double ARM_WRIST_TO_INTAKE_EDGE = Math.sqrt( Math.pow(HardwareConstants.ARM_WRIST_TO_INTAKE_X, 2) +  Math.pow(HardwareConstants.ARM_WRIST_TO_INTAKE_Z, 2));
    
    public static final double SHOOTER_TRIANGLE_ANGLE = Helpers.atan2d( HardwareConstants.ARM_WRIST_TO_SHOOTER_Z, HardwareConstants.ARM_WRIST_TO_SHOOTER_X);
    public static final double INTAKE_TRIANGLE_ANGLE = Helpers.atan2d( HardwareConstants.ARM_WRIST_TO_INTAKE_Z, HardwareConstants.ARM_WRIST_TO_INTAKE_X);

    public static final double ARM_SHOOTER_STARTING_X_POS = 100; 
    public static final double ARM_SHOOTER_STARTING_Z_POS = 100;

    public static final double ARM_TO_CENTER_ROBOT_X_OFFSET = -11.25;
    public static final double ARM_TO_CENTER_ROBOT_Z_OFFSET = 20.75;

    public static final double MIN_USABLE_HEIGHT = 5.5; // bars running across robot height

    //=============================================== Vision ==================================================================//

    public static final double CAMERA_TO_ARM_X_OFFSET = 20;
    public static final double CAMERA_TO_ARM_Z_OFFSET = 20;

    //=============================================== EndEffector =====================================================================//
    
    //TODO get acutal offsets
    public static final double END_EFFECTOR_PIVOT_LENGTH = 2.51; // inches
    public static final double END_EFFECTOR_PIVOT_AXIS_OFFSET = 2.61; //inches

    public static final double SHOOTER_NEAR_FLYWHEEL_TICK_DISTANCE = 1.0;
    public static final double SHOOTER_FAR_FLYWHEEL_TICK_DISTANCE = 1.0;

    public static final double SHOOTER_FLYWHEEL_RADIUS = 2.0;
}
