package frc.robot.mechanisms;

import java.util.List;
import java.util.Set;

import frc.lib.helpers.Graph;
import frc.lib.helpers.GraphNode;
import frc.lib.helpers.Helpers;
import frc.lib.helpers.Pair;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.Point2d;
import frc.robot.HardwareConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;

public class ArmKinematicsCalculator
{
    private static ArmGraph graph;

    private static ArmGraphNode lowerUniversalTransit;
    private static ArmGraphNode startingConfiguration;
    private static ArmGraphNode groundPickup;
    private static ArmGraphNode groundShot;
    private static ArmGraphNode upperUniversalTransit;
    private static ArmGraphNode upperUnivShot;
    private static ArmGraphNode tucked;
    private static ArmGraphNode tuckedTransit;
    private static ArmGraphNode tuckedGroundTransit;
    private static ArmGraphNode tuckedUnderTransit;
    private static ArmGraphNode sourcePickup;
    private static ArmGraphNode ampScore;
    private static ArmGraphNode upperIntakeFlipped;
    private static ArmGraphNode trapIntermediate;
    private static ArmGraphNode upperObtuseWrist;
    private static ArmGraphNode ampScoreOuttakeUp;

    static
    {
        ArmKinematicsCalculator.graph = new ArmGraph();

        // intialize all of the graph nodes

        // lower-universal positions:
        ArmKinematicsCalculator.lowerUniversalTransit = ArmKinematicsCalculator.graph.createNode(
            "lowerUniversalTransit",
            TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL,
            TuningConstants.ARM_SHOULDER_UNIVERSAL_DELTA,
            TuningConstants.ARM_WRIST_POSITION_LOWER_UNIVERSAL_MIN,
            TuningConstants.ARM_WRIST_POSITION_LOWER_UNIVERSAL_MAX);
        ArmKinematicsCalculator.startingConfiguration = ArmKinematicsCalculator.graph.createNode(
            "StartingConfiguration",
            TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION,
            TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);
        ArmKinematicsCalculator.groundPickup = ArmKinematicsCalculator.graph.createNode(
            "groundPickup",
            TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL,
            TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP);
        ArmKinematicsCalculator.groundShot = ArmKinematicsCalculator.graph.createNode(
            "groundShot",
            TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL,
            TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT);

        // upper-universal positions:
        ArmKinematicsCalculator.upperUniversalTransit = ArmKinematicsCalculator.graph.createNode(
            "upperUniversalTransit",
            TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL,
            TuningConstants.ARM_SHOULDER_UNIVERSAL_DELTA,
            TuningConstants.ARM_WRIST_POSITION_UPPER_UNIVERSAL_MIN,
            TuningConstants.ARM_WRIST_POSITION_UPPER_UNIVERSAL_MAX);
        ArmKinematicsCalculator.upperUnivShot = ArmKinematicsCalculator.graph.createNode(
            "upperUniversalShot",
            TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL,
            TuningConstants.ARM_WRIST_POSITION_UPPER_UNIVERSAL_SHOT);
        ArmKinematicsCalculator.upperIntakeFlipped = ArmKinematicsCalculator.graph.createNode(
            "upperIntakeFlipped",
            TuningConstants.ARM_SHOULDER_POSITION_INTAKE_FLIPPED,
            TuningConstants.ARM_WRIST_POSITION_INTAKE_FLIPPED);
        ArmKinematicsCalculator.upperObtuseWrist = ArmKinematicsCalculator.graph.createNode(
            "upperObtuseWrist",
            TuningConstants.ARM_SHOULDER_POSITION_INTAKE_OBTUSE,
            TuningConstants.ARM_WRIST_POSITION_INTAKE_OBTUSE);

        ArmKinematicsCalculator.tucked = ArmKinematicsCalculator.graph.createNode(
            "tucked",
            TuningConstants.ARM_SHOULDER_POSITION_TUCKED,
            TuningConstants.ARM_WRIST_POSITION_TUCKED_SHOT);

        ArmKinematicsCalculator.tuckedTransit = ArmKinematicsCalculator.graph.createNode(
            "tuckedTransit",
            TuningConstants.ARM_SHOULDER_POSITION_TUCKED_TRANSIT,
            TuningConstants.ARM_WRIST_POSITION_TUCKED_TRANSIT);

        ArmKinematicsCalculator.tuckedGroundTransit = ArmKinematicsCalculator.graph.createNode(
            "tuckedGroundTransit", 
            TuningConstants.ARM_SHOULDER_POSITION_TUCKED_GROUND_TRANSIT, 
            TuningConstants.ARM_WRIST_POSITION_TUCKED_GROUND_TRANSIT);

        ArmKinematicsCalculator.tuckedUnderTransit = ArmKinematicsCalculator.graph.createNode(
            "tuckedUnderTransit",
            TuningConstants.ARM_SHOULDER_POSITION_TUCKED_UNDER_TRANSIT,
            TuningConstants.ARM_WRIST_POSITION_TUCKED_UNDER_TRANSIT);

        ArmKinematicsCalculator.sourcePickup = ArmKinematicsCalculator.graph.createNode(
            "sourcePickup",
            TuningConstants.ARM_SHOULDER_POSITION_SOURCE_PICKUP,
            TuningConstants.ARM_WRIST_POSITION_SOURCE_PICKUP);

        ArmKinematicsCalculator.ampScore = ArmKinematicsCalculator.graph.createNode(
            "ampScore",
            TuningConstants.ARM_SHOULDER_POSITION_AMP_SCORE,
            TuningConstants.ARM_WRIST_POSITION_AMP_SCORE);

        ArmKinematicsCalculator.ampScoreOuttakeUp = ArmKinematicsCalculator.graph.createNode(
            "ampScoreOuttakeUp",
            TuningConstants.ARM_SHOULDER_POSITION_AMP_OUTTAKE,
            TuningConstants.ARM_WRIST_POSITION_AMP_OUTTAKE);

        ArmKinematicsCalculator.trapIntermediate = ArmKinematicsCalculator.graph.createNode(
            "trapIntermediate",
            TuningConstants.ARM_SHOULDER_POSITION_TRAP_INTERMEDIATE,
            TuningConstants.ARM_WRIST_POSITION_TRAP_INTERMEDIATE);

        // create all of the links between the nodes
        // links between each of the the lower-universal node combinations
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.startingConfiguration,
            ArmKinematicsCalculator.groundPickup,
            TuningConstants.STARTUP_AND_GROUND_PICKUP_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.startingConfiguration,
            ArmKinematicsCalculator.groundShot,
            TuningConstants.STARTUP_AND_GROUND_SHOT_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.groundPickup,
            ArmKinematicsCalculator.groundShot,
            TuningConstants.GROUND_PICKUP_AND_GROUND_SHOT_WEIGHT);

        // lower-universal transit to lower-univeral nodes
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.lowerUniversalTransit,
            ArmKinematicsCalculator.startingConfiguration,
            TuningConstants.LOWER_UNIVERSAL_TRANSIT_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.lowerUniversalTransit,
            ArmKinematicsCalculator.groundShot,
            TuningConstants.LOWER_UNIVERSAL_TRANSIT_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.lowerUniversalTransit,
            ArmKinematicsCalculator.groundPickup,
            TuningConstants.LOWER_UNIVERSAL_TRANSIT_WEIGHT);

        // links between each of the upper-universal node combinations
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.upperIntakeFlipped,
            ArmKinematicsCalculator.upperUnivShot,
            TuningConstants.UPPER_INTAKE_FLIPPED_AND_UPPER_UNIV_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.upperUnivShot,
            ArmKinematicsCalculator.upperObtuseWrist,
            TuningConstants.UPPER_UNIV_AND_OBTUSE_WRIST_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.upperObtuseWrist,
            ArmKinematicsCalculator.upperIntakeFlipped,
            TuningConstants.UPPER_INTAKE_FLIPPED_AND_UPPER_OBTUSE_WEIGHT);

        ArmKinematicsCalculator.graph.connectBidirectional(
            ampScoreOuttakeUp,
            startingConfiguration,
            1.0);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ampScoreOuttakeUp,
            groundShot,
            1.0);

        // upper-universal transit to upper-univeral nodes
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.upperUniversalTransit,
            ArmKinematicsCalculator.upperUnivShot,
            TuningConstants.UPPER_UNIVERSAL_TRANSIT_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.upperUniversalTransit,
            ArmKinematicsCalculator.upperIntakeFlipped,
            TuningConstants.UPPER_UNIVERSAL_TRANSIT_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.upperUniversalTransit,
            ArmKinematicsCalculator.upperObtuseWrist,
            TuningConstants.UPPER_UNIVERSAL_TRANSIT_WEIGHT);

        // special IK-friendly transition from upper universal (shot) to ground transit
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.tuckedTransit,
            ArmKinematicsCalculator.upperUnivShot,
            TuningConstants.UPPER_UNIV_AND_TUCKED_TRANSIT_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.tuckedGroundTransit,
            ArmKinematicsCalculator.groundPickup,
            TuningConstants.GROUND_PICKUP_AND_TUCKED_GROUND_TRANSIT_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.tuckedGroundTransit,
            ArmKinematicsCalculator.groundShot,
            TuningConstants.GROUND_SHOT_AND_TUCKED_GROUND_TRANSIT_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.tuckedGroundTransit,
            ArmKinematicsCalculator.tuckedTransit,
            TuningConstants.TUCKED_TRANSIT_AND_TUCKED_GROUND_TRANSIT_WEIGHT);

        // Lower quartile friendly values
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.startingConfiguration,
            ArmKinematicsCalculator.sourcePickup,
            TuningConstants.STARTUP_AND_SOURCE_PICKUP_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.startingConfiguration,
            ArmKinematicsCalculator.upperIntakeFlipped,
            TuningConstants.STARTUP_AND_UPPER_INTAKE_FLIPPED_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.sourcePickup,
            ArmKinematicsCalculator.upperIntakeFlipped,
            TuningConstants.SOURCE_PICKUP_AND_UPPER_INTAKE_FLIPPED_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.upperIntakeFlipped,
            ArmKinematicsCalculator.trapIntermediate,
            TuningConstants.UPPER_INTAKE_FLIPPED_AND_TRAP_INTER_WEIGHT);

        // path to tucked
        ArmKinematicsCalculator.graph.connect(
            ArmKinematicsCalculator.tuckedTransit,
            ArmKinematicsCalculator.tucked,
            TuningConstants.TUCKED_TRANSIT_TO_TUCKED_WEIGHT);
        
        // Doing with IK??
        // ArmKinematicsCalculator.graph.connect(
            // ArmKinematicsCalculator.groundPickup,
            // ArmKinematicsCalculator.upperUnivShot,
            // TuningConstants.UPPER_UNIV_AND_GROUND_PICKUP_WEIGHT);

        // Upper quartile friendly values    
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.ampScore,
            ArmKinematicsCalculator.upperUnivShot,
            TuningConstants.AMP_SCORE_AND_UPPER_UNIV_SHOT_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.ampScore,
            ArmKinematicsCalculator.upperObtuseWrist,
            TuningConstants.AMP_SCORE_AND_OBTUSE_WRIST_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.upperObtuseWrist,
            ArmKinematicsCalculator.tuckedTransit,
            TuningConstants.OBTUSE_WRIST_AND_TUCKED_TRANSIT_WEIGHT);
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.upperObtuseWrist,
            ArmKinematicsCalculator.tucked,
            TuningConstants.OBTUSE_WRIST_AND_TUCKED_WEIGHT);

        // Save us for hiting robot!!
        ArmKinematicsCalculator.graph.connect(
            ArmKinematicsCalculator.tuckedUnderTransit,
            ArmKinematicsCalculator.tuckedTransit,
            TuningConstants.TUCKED_UNDER_TO_TUCKED_TRANSIT_WEIGHT);
        
        // Quickest path to tucked from ground pickup
        ArmKinematicsCalculator.graph.connectBidirectional(
            ArmKinematicsCalculator.tucked,
            ArmKinematicsCalculator.tuckedTransit,
            TuningConstants.TUCKED_TO_TUCKED_TRANSIT_WEIGHT);
    }

    public enum ExtensionType
    {
        None,
        Back,
        Robot,
        Ground,
        TopCrazy,
        TopBoth,
        TopIntakeSide,
        TopShooterSide,
        TopNone,
        FrontBoth,
        FrontIntakeTop,
        FrontIntakeBottom,
        FrontNone,
    }

    private double desiredWristAngle; // tester
    private double desiredShoulderAngle;
    private ExtensionType extensionType;

    private final double L1 = HardwareConstants.ARM_HUMERUS_LENGTH; // Shoulder pivot to wrist pivot distance
    private final double L2 = HardwareConstants.ARM_WRIST_TO_SHOOTER_EDGE; // wrist to shooter top
    private final double L3 = HardwareConstants.ARM_WRIST_TO_INTAKE_EDGE; // wirst to intake top
    private final double L2x = HardwareConstants.ARM_WRIST_TO_SHOOTER_X; // wrist to shooter bottom
    private final double L2z = HardwareConstants.ARM_WRIST_TO_INTAKE_Z; // intake height
    private final double L3x = HardwareConstants.ARM_WRIST_TO_INTAKE_X; // wrist to intake bottom
    private final double L3z = HardwareConstants.ARM_WRIST_TO_SHOOTER_Z; // shooter height

    private double shoulderJointAbsPosX = HardwareConstants.ARM_TO_CENTER_ROBOT_X_OFFSET;
    private double shoulderJointAbsPosZ = HardwareConstants.ARM_TO_CENTER_ROBOT_Z_OFFSET;
    private double wristAbsPosX = this.shoulderJointAbsPosX + Helpers.cosd(this.theta_1) * L1;
    private double wristAbsPosZ = this.shoulderJointAbsPosZ + Helpers.sind(this.theta_1) * L1;
    private double shooterBottomAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_4) * L2x;
    private double shooterBottomAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_4) * L2x;
    private double shooterTopAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_8) * L2;
    private double shooterTopAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_8) * L2;
    private double intakeBottomAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_5) * L3x;
    private double intakeBottomAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_5) * L3x;
    private double intakeTopAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_9) * L3;
    private double intakeTopAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_9) * L3;

    // IK VARIABLES
    private double theta_1; // Horizontal to shoulder ABS angle
    private double theta_2; // Wrist to Shoulder rel angle
    private double theta_3; // 180 - theta_1
    private double theta_4; // Horizontal to Wrist (shooter bottom) ABS angle
    private double theta_5; // Horizontal to intake bottom ABS angle
    private double theta_6 = HardwareConstants.SHOOTER_TRIANGLE_ANGLE; // Shooter triangle angle
    private double theta_7 = HardwareConstants.INTAKE_TRIANGLE_ANGLE; // Intake triangle angle
    private double theta_8; // Horizontal to shooter top ABS angle
    private double theta_9; // Horizontal to intake top ABS angle

    private boolean stuckInPosition;
    private boolean hittingRobot;
    private boolean fixedWithIK;

    public ArmKinematicsCalculator(double shoulderPosition, double wristPosition)
    {
        this.calculate(shoulderPosition, wristPosition);
        this.stuckInPosition = false;
        this.hittingRobot = false;
        this.fixedWithIK = false;
    }

    public void logValues(ILogger logger)
    {
        logger.logBoolean(LoggingKey.ArmExtensionBreaking, this.stuckInPosition);
        logger.logBoolean(LoggingKey.HittingRobot, this.hittingRobot);
        logger.logBoolean(LoggingKey.FixedWithIK, this.fixedWithIK);
        logger.logString(LoggingKey.ExtensionType, this.extensionType.toString());

        logger.logNumber(LoggingKey.IntakeTopAbsX, this.intakeTopAbsPosX);
        logger.logNumber(LoggingKey.IntakeTopAbsZ, this.intakeTopAbsPosZ);
        logger.logNumber(LoggingKey.IntakeBottomAbsX, this.intakeBottomAbsPosX);
        logger.logNumber(LoggingKey.IntakeBottomAbsZ, this.intakeBottomAbsPosZ);
        logger.logNumber(LoggingKey.ShooterTopAbsX, this.shooterTopAbsPosX);
        logger.logNumber(LoggingKey.ShooterTopAbsZ, this.shooterTopAbsPosZ);
        logger.logNumber(LoggingKey.ShooterBottomAbsX, this.shooterBottomAbsPosX);
        logger.logNumber(LoggingKey.ShooterBottomAbsZ, this.shooterBottomAbsPosZ);
        logger.logNumber(LoggingKey.WristAbsX, this.wristAbsPosX);
        logger.logNumber(LoggingKey.WristAbsZ, this.wristAbsPosZ);
        logger.logNumber(LoggingKey.ShoulderIKDesired, this.desiredShoulderAngle);
        logger.logNumber(LoggingKey.WristIKDesired, this.desiredWristAngle);
    }

    /**
     * Calculate the various different values based on the provided shoulder and wrist angles
     * @param shoulderAngle
     * @param wristAngle
     */
    public void calculate(double shoulderAngle, double wristAngle)
    {
        this.theta_1 = shoulderAngle;
        this.theta_2 = wristAngle;
        this.theta_3 = 180.0 - this.theta_1;
        this.theta_4 = 360 - this.theta_3 - this.theta_2;
        this.theta_5 = this.theta_4 - 180.0;
        this.theta_8 = this.theta_4 - this.theta_6;
        this.theta_9 = this.theta_5 + this.theta_7;

        this.shoulderJointAbsPosX = HardwareConstants.ARM_TO_CENTER_ROBOT_X_OFFSET;
        this.shoulderJointAbsPosZ = HardwareConstants.ARM_TO_CENTER_ROBOT_Z_OFFSET;
        this.wristAbsPosX = this.shoulderJointAbsPosX + Helpers.cosd(this.theta_1) * L1;
        this.wristAbsPosZ = this.shoulderJointAbsPosZ + Helpers.sind(this.theta_1) * L1;
        this.shooterBottomAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_4) * L2x;
        this.shooterBottomAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_4) * L2x;
        this.shooterTopAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_8) * L2;
        this.shooterTopAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_8) * L2;
        this.intakeBottomAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_5) * L3x;
        this.intakeBottomAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_5) * L3x;
        this.intakeTopAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_9) * L3;
        this.intakeTopAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_9) * L3;
    }

    /**
     * Calculate the arm limits based on the current desired shoulder and wrist positions
     * @param currentDesiredShoulderPosition the desired position for the shoulder to be moved to during this next update cycle
     * @param currentDesiredWristPosition the desired position for the wrist to be moved to during this next update cycle
     * @param kinematicsLimitedAngles the angles that we are limited to (in: last legal positions, out: updated desired positions)
     * @return true if we are using a different position than the "desired" ones, false if we are using the desired position
     */
    public boolean calculateArmLimits(
        double currentDesiredShoulderPosition,
        double currentDesiredWristPosition,
        Pair<Double, Double> kinematicsLimitedAngles)
    {
        this.calculate(currentDesiredShoulderPosition, currentDesiredWristPosition);

        this.fixedWithIK = false;
        this.hittingRobot = false;
        this.stuckInPosition = false;
        this.extensionType = ExtensionType.None;

        boolean extensionTop =
            this.intakeTopAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT ||
            this.shooterTopAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT;
        boolean extensionFront =
            this.intakeTopAbsPosX > HardwareConstants.MAX_ROBOT_EXTENSION + HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0 ||
            this.intakeBottomAbsPosX > HardwareConstants.MAX_ROBOT_EXTENSION + HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0;
        boolean extensionBack =
            -this.intakeBottomAbsPosX > HardwareConstants.MAX_ROBOT_EXTENSION + HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0 ||
            -this.intakeTopAbsPosX > HardwareConstants.MAX_ROBOT_EXTENSION + HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0;

        // Instant limiting
        if (extensionBack)
        {
            this.extensionType = ExtensionType.Back;
            this.stuckInPosition = true;
            return true;
        }

        if (this.intakeBottomAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT || this.shooterBottomAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT)
        {
            this.extensionType = ExtensionType.TopCrazy;
            this.stuckInPosition = true;
            return true;
        }

        // hiting robot
        if ((this.intakeTopAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.intakeTopAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0) ||
            (this.intakeBottomAbsPosZ > 0.0 && this.intakeBottomAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.intakeBottomAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0) ||
            (this.shooterTopAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.shooterTopAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0) ||
            (this.shooterBottomAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.shooterBottomAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0))
        {
            // special-case for the lower universal (shoulder at base), which we know is legal through the range of motion of the wrist from
            // its starting configuration to the ground pickup location
            // we're not hitting the robot, the Kinematics are just impossibly weird to work with
            if (this.theta_2 >= TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION &&
                this.theta_2 <= TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP_IK &&
                Helpers.RoughEquals(this.theta_1, TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, 5.0))
            {
                // neither fixed nor stuck at this position - use the new angles
                kinematicsLimitedAngles.set(
                    currentDesiredShoulderPosition,
                    currentDesiredWristPosition);
                return false;
            }
            else
            {
                this.extensionType = ExtensionType.Robot;
                this.hittingRobot = true;
                this.stuckInPosition = true;
                return true;
            }
        }

        // hitting ground
        if ((this.intakeTopAbsPosZ < -5.0) || (this.intakeBottomAbsPosZ < -5.0))
        {
            // special-case for the lower universal (shoulder at base), which we know is legal through the range of motion of the wrist from
            // its starting configuration to the ground pickup location
            // we're not hitting the robot, the Kinematics are just impossibly weird to work with
            if (this.theta_2 >= TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION &&
                this.theta_2 <= TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP &&
                Helpers.RoughEquals(this.theta_1, TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, 5.0))
            {
                // neither fixed nor stuck at this position - use the new angles
                kinematicsLimitedAngles.set(
                    currentDesiredShoulderPosition,
                    currentDesiredWristPosition);
                return false;
            }
            else
            {
                this.extensionType = ExtensionType.Ground;
                this.stuckInPosition = true;
                return true;
            }
        }

        // continous limiting top
        if (extensionTop)
        {
            boolean intakeSide = intakeTopAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT;
            boolean shooterSide = shooterTopAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT;
            double desiredDistance = HardwareConstants.MAX_ROBOT_HEIGHT - this.wristAbsPosZ;

            if (intakeSide && shooterSide)
            {
                this.extensionType = ExtensionType.TopBoth;
                this.stuckInPosition = true;
                return true;
            }

            if (intakeSide && intakeTopAbsPosX > wristAbsPosX)
            {
                double temp_theta_9 = Helpers.asind(desiredDistance / L3); // meant to return a positive value, negative added since asin(0.5) is negative
                double temp_theta_4 = temp_theta_9 - this.theta_7 + 180.0;
                double temp_wrist_angle = 180 + this.theta_1 - temp_theta_4;

                kinematicsLimitedAngles.set(
                    currentDesiredShoulderPosition,
                    temp_wrist_angle);
                this.fixedWithIK = true;
                this.extensionType = ExtensionType.TopIntakeSide;
                return true;
            }

            if (shooterSide && shooterTopAbsPosX > wristAbsPosX)
            {
                double temp_theta_8 = Helpers.asind(desiredDistance / L2); // meant to return a positive value, negative added since asin(0.5) is negative
                double temp_theta_4 = temp_theta_8 + this.theta_6;
                double temp_wrist_angle = 180 + this.theta_1 - temp_theta_4;

                kinematicsLimitedAngles.set(
                    currentDesiredShoulderPosition,
                    temp_wrist_angle);
                this.fixedWithIK = true;
                this.extensionType = ExtensionType.TopShooterSide;
                return true;
            }

            // cannot fix
            this.extensionType = ExtensionType.TopNone;
            this.stuckInPosition = true;
            return true;
        }

        // continous limiting front
        if (extensionFront)
        {
            boolean intakeTop = intakeTopAbsPosX > HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0 + HardwareConstants.MAX_ROBOT_EXTENSION;
            boolean intakeBottom = intakeBottomAbsPosX > HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0 + HardwareConstants.MAX_ROBOT_EXTENSION;
            double desiredDistance = HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0 + HardwareConstants.MAX_ROBOT_EXTENSION - this.wristAbsPosX;

            if (intakeTop && intakeBottom)
            {
                this.extensionType = ExtensionType.FrontBoth;
                this.stuckInPosition = true;
                return true;
            }

            if (intakeTop && intakeTopAbsPosZ < wristAbsPosZ)
            {
                double temp_theta_9 = -Helpers.acosd(desiredDistance / L3); // meant to return a negative value, negative added since acos(0.5) is positive
                double temp_theta_4 = temp_theta_9 - this.theta_7 + 180.0;
                double temp_wrist_angle = 180 + this.theta_1 - temp_theta_4;

                kinematicsLimitedAngles.set(
                    currentDesiredShoulderPosition,
                    temp_wrist_angle);
                this.extensionType = ExtensionType.FrontIntakeTop;
                this.fixedWithIK = true;
                return true;
            }

            if (intakeBottom)
            {
                double temp_theta_5 = Helpers.acosd(desiredDistance / L3x);
                double temp_theta_4 = temp_theta_5 + 180.0;
                double temp_wrist_angle = 180 + this.theta_1 - temp_theta_4;

                kinematicsLimitedAngles.set(
                    currentDesiredShoulderPosition,
                    temp_wrist_angle);
                this.fixedWithIK = true;
                this.extensionType = ExtensionType.FrontIntakeBottom;
                return true;
            }

            // cannot fix
            this.extensionType = ExtensionType.FrontNone;
            this.stuckInPosition = true;
            return true;
        }

        // neither fixed nor stuck at this position - use the new angles
        kinematicsLimitedAngles.set(
            currentDesiredShoulderPosition,
            currentDesiredWristPosition);
        return false;
    }

    public double getTheta1()
    {
        return this.theta_1;
    }

    public double getTheta2()
    {
        return this.theta_2;
    }

    public double getAbsoluteAngleOfShot()
    {
        return this.theta_4;
    }

    public double switchToTheta2(double desiredAbsWrist)
    {
        return 180 + this.theta_1 - desiredAbsWrist;
    }

    public double[] getWristJointAbsPosition()
    {
        double[] absWristPosition = new double[2];
        absWristPosition[0] = this.wristAbsPosX;
        absWristPosition[1] = this.wristAbsPosZ;

        return absWristPosition;
    }

    public boolean getStuckInPosition()
    {
        return this.stuckInPosition;
    }

    public ExtensionType getExtensionType()
    {
        return this.extensionType;
    }

    public Point2d getShooterBottomAbsPos()
    {
        return new Point2d(this.shooterBottomAbsPosX, this.shooterBottomAbsPosZ);
    }

    public Point2d getShooterTopAbsPos()
    {
        return new Point2d(this.shooterTopAbsPosX, this.shooterTopAbsPosZ);
    }

    public Point2d getIntakeBottomAbsPos()
    {
        return new Point2d(this.intakeBottomAbsPosX, this.intakeBottomAbsPosZ);
    }

    public Point2d getIntakeTopAbsPos()
    {
        return new Point2d(this.intakeTopAbsPosX, this.intakeTopAbsPosZ);
    }

    public static ArmGraphNode getClosestArmNode(double shoulderPos, double wristPos)
    {
        double diff = Double.POSITIVE_INFINITY;
        ArmGraphNode nearest = null;

        for (ArmGraphNode curr : ArmKinematicsCalculator.graph.getNodes())
        {
            double newDiff = curr.calculateDistance(shoulderPos, wristPos);
            if (diff > newDiff)
            {
                nearest = curr;
                diff = newDiff;
            }
        }

        return nearest;
    }

    public static List<ArmGraphNode> getOptimalPath(ArmGraphNode startArmGraphNode, ArmGraphNode goalArmGraphNode)
    {
        return ArmKinematicsCalculator.graph.getOptimalPath(startArmGraphNode, goalArmGraphNode);
    }

    public static Set<ArmGraphNode> getAllGraphNodes()
    {
        return ArmKinematicsCalculator.graph.getNodes();
    }

    private static class ArmGraph extends Graph<ArmGraphNode>
    {
        public ArmGraph()
        {
            super();
        }

        public ArmGraphNode createNode(String name, double shoulderAngle, double wristAngle)
        {
            ArmGraphNode node = new ArmGraphNode(name, shoulderAngle, wristAngle);
            this.addNode(node);
            return node;
        }

        public ArmGraphNode createNode(String name, double shoulderAngle, double shoulderDelta, double wristRangeMin, double wristRangeMax)
        {
            ArmGraphNode node = new ArmGraphNode(name, shoulderAngle, shoulderDelta, wristRangeMin, wristRangeMax);
            this.addNode(node);
            return node;
        }
    }

    public static class ArmGraphNode extends GraphNode
    {
        private final String name;
        private final Double wristAngle;
        private final double shoulderAngle;

        private final double wristUniversalRangeMin;
        private final double wristUniversalRangeMax;
        private final double shoulderDelta;

        public ArmGraphNode(String name, double shoulderAngle, double wristAngle)
        {
            this(name, shoulderAngle, wristAngle, 0.0, 0.0, 0.0);
        }

        public ArmGraphNode(String name, double shoulderAngle, double shoulderDelta, double wristRangeMin, double wristRangeMax)
        {
            this(name, shoulderAngle, null, shoulderDelta, wristRangeMin, wristRangeMax);
        }

        private ArmGraphNode(String name, double shoulderAngle, Double wristAngle, double shoulderDelta, double wristRangeMin, double wristRangeMax)
        {
            super();

            this.name = name;
            this.shoulderAngle = shoulderAngle;
            this.wristAngle = wristAngle;
            this.shoulderDelta = shoulderDelta;
            this.wristUniversalRangeMin = wristRangeMin;
            this.wristUniversalRangeMax = wristRangeMax;
        }

        public double calculateDistance(double shoulderPos, double wristPos)
        {
            if (this.wristAngle != null)
            {
                return
                    Math.abs(this.shoulderAngle - shoulderPos) * TuningConstants.ARM_SHOULDER_WEIGHT_MULTIPLIER +
                    Math.abs(this.wristAngle - wristPos) * TuningConstants.ARM_WRIST_WEIGHT_MULTIPLIER;
            }

            if (Helpers.WithinDelta(shoulderPos, this.shoulderAngle, this.shoulderDelta) &&
                Helpers.WithinRange(wristPos, this.wristUniversalRangeMin, this.wristUniversalRangeMax))
            {
                // claim that we are just outside the "range" of any node
                return TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD * TuningConstants.ARM_SHOULDER_WEIGHT_MULTIPLIER +
                    TuningConstants.ARM_WRIST_GOAL_THRESHOLD * TuningConstants.ARM_WRIST_WEIGHT_MULTIPLIER;
            }

            return Double.MAX_VALUE;
        }

        public boolean isUniversal()
        {
            return this.wristAngle == null;
        }

        public String getName()
        {
            return this.name;
        }

        public double getShoulderAngle()
        {
            return this.shoulderAngle;
        }

        public Double getWristAngle()
        {
            return this.wristAngle;
        }

        @Override
        public String toString()
        {
            if (this.wristAngle == null)
            {
                return String.format("%s (%.2f, *)", this.name, this.shoulderAngle);
            }

            return String.format("%s (%.2f, %.2f)", this.name, this.shoulderAngle, this.wristAngle);
        }
    }
}
