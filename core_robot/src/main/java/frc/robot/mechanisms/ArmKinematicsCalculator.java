package frc.robot.mechanisms;

import frc.lib.helpers.Helpers;
import frc.lib.helpers.Pair;
import frc.lib.robotprovider.ILogger;
import frc.robot.HardwareConstants;
import frc.robot.LoggingKey;

public class ArmKinematicsCalculator
{
    private double desiredWristAngle; // tester
    private double desiredShoulderAngle;
    private String extensionType = "None";

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
        logger.logString(LoggingKey.ExtensionType, this.extensionType);

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
        this.extensionType = "None";

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
            this.extensionType = "Back";
            this.stuckInPosition = true;
            return true;
        }

        if (this.intakeBottomAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT || this.shooterBottomAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT)
        {
            this.extensionType = "Top-Crazy";
            this.stuckInPosition = true;
            return true;
        }

        // hiting robot
        if ( (this.intakeTopAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.intakeTopAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0)
            || (this.intakeBottomAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.intakeBottomAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0)
            || (this.shooterTopAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.shooterTopAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0)
            || (this.shooterBottomAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.shooterBottomAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0))
        {
            this.extensionType = "Robot";
            this.hittingRobot = true;
            this.stuckInPosition = true;
            return true;
        }

        // hitting ground
        if ( (this.intakeTopAbsPosZ < -2.0) || (this.intakeBottomAbsPosZ < -2.0))
        {
            this.extensionType = "Ground";
            this.stuckInPosition = true;
            return true;
        }

        // continous limiting top
        if (extensionTop)
        {
            boolean intakeSide = intakeTopAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT;
            boolean shooterSide = shooterTopAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT;
            double desiredDistance = HardwareConstants.MAX_ROBOT_HEIGHT - this.wristAbsPosZ;

            if (intakeSide && shooterSide)
            {
                this.extensionType = "Top-Both";
                this.stuckInPosition = true;
                return true;
            }

            if (intakeSide && intakeTopAbsPosX > wristAbsPosX)
            {
                double temp_theta_9 = -Helpers.asind(desiredDistance / L3); // meant to return a positive value, negative added since asin(0.5) is negative
                double temp_theta_4 = temp_theta_9 - this.theta_7 + 180.0;
                double temp_wrist_angle = 180 + this.theta_1 - temp_theta_4;

                kinematicsLimitedAngles.set(
                    currentDesiredShoulderPosition,
                    temp_wrist_angle);
                this.fixedWithIK = true;
                return true;
            }

            if (shooterSide && shooterTopAbsPosX > wristAbsPosX)
            {
                double temp_theta_8 = -Helpers.asind(desiredDistance / L2); // meant to return a positive value, negative added since asin(0.5) is negative
                double temp_theta_4 = temp_theta_8 + this.theta_6;
                double temp_wrist_angle = 180 + this.theta_1 - temp_theta_4;

                kinematicsLimitedAngles.set(
                    currentDesiredShoulderPosition,
                    temp_wrist_angle);
                this.fixedWithIK = true;
                return true;
            }

            // cannot fix
            this.extensionType = "Top-None";
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
                this.extensionType = "Front-Both";
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
                return true;
            }

            // cannot fix
            this.extensionType = "Front-None";
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
}
