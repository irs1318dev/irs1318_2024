package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.FieldConstants;
import frc.robot.HardwareConstants;
import frc.lib.driver.IControlTask;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that calculates desired angle to shoot note
 * @author FWJK35 (Calvin), Will
 */
public class VisionShooterAimMathTask extends ControlTaskBase
{
    public static IControlTask createShootMacroTask(boolean continuous)
    {
        if (continuous)
        {
            return ConcurrentTask.AnyTasks(
                new RumbleTask(),
                new ShooterSpinTask(TuningConstants.SHOOT_VISION_SPEED, 15.0),
                SequentialTask.Sequence(
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                    ConcurrentTask.AllTasks(
                        new VisionSingleTurningTask(VisionSingleTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear),
                        new VisionShooterAimMathTask(false, true)),
                    ConcurrentTask.AnyTasks(
                        new VisionContinuousTurningTask(VisionContinuousTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear, true),
                        new VisionShooterAimMathTask(true, true),
                        new FeedRingTask(true, 5.0))));
        }

        return ConcurrentTask.AnyTasks(
            new RumbleTask(),
            new ShooterSpinTask(TuningConstants.SHOOT_VISION_SPEED, 15.0),
            SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new VisionSingleTurningTask(VisionSingleTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear),
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                new VisionShooterAimMathTask(false, true),
                new FeedRingTask(true, 5.0))));
    }

    private final boolean continuous;
    private final boolean useMaxVelocity;

    private ArmMechanism armMechanism;
    private OffboardVisionManager visionManager;

    private double desiredAngle;

    private double pivotToTargetXDist;
    private double pivotToTargetYDist;

    private int noAprilTags;

    private double desiredVelocity;

    public VisionShooterAimMathTask()
    {
        this(false, true);
    }

    public VisionShooterAimMathTask(boolean continuous, boolean useMaxVelocity)
    {
        if (!useMaxVelocity)
        {
            ExceptionHelpers.Assert(continuous, "non-max velocity mode requires continuous mode");
        }

        this.continuous = continuous;
        this.useMaxVelocity = useMaxVelocity;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.armMechanism = this.getInjector().getInstance(ArmMechanism.class);
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);

        for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, op == DigitalOperation.VisionFindSpeakerAprilTagRear);
        }
    }

    /*
     * Update the current task and controls
     */
    @Override
    public void update()
    {
        Double visionX = visionManager.getAprilTagXOffset();
        Double visionY = visionManager.getAprilTagYOffset();
        if (visionX == null || visionY == null)
        {
            this.noAprilTags++;
        }
        else
        {
            // use both x and y offsets to calculate the distance in case we are not facing it _exactly_
            double distance = Math.sqrt(visionX * visionX + visionY * visionY);

            this.noAprilTags = 0;

            double distToTargetX = distance
                + FieldConstants.APRILTAG_TO_SPEAKER_TARGET_X
                + 13.0; // this.armMechanism.getWristJointAbsPosition()[0];
            double distToTargetY = 81.0 - 15.0; // this.armMechanism.getWristJointAbsPosition()[1];

            pivotToTargetXDist = distToTargetX;
            pivotToTargetYDist = distToTargetY;
            double absAngle = getAngleSimple(distToTargetX, distToTargetY, TuningConstants.SHOOTER_MAX_VELOCITY);
            this.setDesiredAngleFromXYOffsets(distToTargetX, distToTargetY);

            this.setAnalogOperationState(AnalogOperation.ArmAbsWristAngle, this.desiredAngle);
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);  

            // Assume the speed is being set elsewhere when useMaxVelocity is true...
            if (!this.useMaxVelocity && this.continuous)
            {
                this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, this.desiredVelocity);
                this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, this.desiredVelocity);
            }
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, false);
        }

        this.setAnalogOperationState(AnalogOperation.ArmAbsWristAngle, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);

        if (!this.useMaxVelocity && this.continuous)
        {
            this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, TuningConstants.MAGIC_NULL_VALUE);
            this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, TuningConstants.MAGIC_NULL_VALUE);
        }
    }

    @Override
    public boolean shouldCancel()
    {
        return this.noAprilTags > 20;
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.continuous)
        {
            return false;
        }

        double currPos = this.armMechanism.getAbsoluteAngleOfShot();
        return Helpers.RoughEquals(currPos, this.desiredAngle, TuningConstants.ARM_WRIST_GOAL_THRESHOLD);
    }

    private void setDesiredAngleFromXYOffsets(double pivotToTargetXDist, double pivotToTargetYDist) {
        double leftBound = 0;
        double rightBound = 90;
        double midpoint = (leftBound + rightBound) * 0.5;

        //find the intersection between required velocity for upwards angle
        //and velocity needed to intersect target
        for (int i = 0; i < TuningConstants.ANGLE_FINDING_ITERATIONS; i++) {
            double resultantValue;
            resultantValue = getResultantAngleIntersection(midpoint);
            if (resultantValue < 0) {
                rightBound = midpoint;
            }
            else {
                leftBound = midpoint;
            }
            midpoint = (leftBound + rightBound) * 0.5;
        }

        // if it should fire with the max velocity, calculate that angle
        if (this.useMaxVelocity)
        {
            leftBound = 0;
            rightBound = midpoint;
            midpoint = (leftBound + rightBound) * 0.5;
            for (int i = 0; i < TuningConstants.ANGLE_FINDING_ITERATIONS; i++) {
                double resultantValue;
                resultantValue = getResultantAngleMaxVelocity(midpoint);
                if (resultantValue < 0) {
                    rightBound = midpoint;
                }
                else {
                    leftBound = midpoint;
                }
                midpoint = (leftBound + rightBound) * 0.5;
            }
        }

        this.desiredAngle = midpoint + TuningConstants.SHOOTER_FINAL_ANGLE_OFFSET;
        this.desiredVelocity = getVelocityFromAngleTarget(desiredAngle);
    }

    /*
     * gets the required velocity to hit the target with a given angle
     */
    private double getVelocityFromAngleTarget(double theta) {
        theta *= Helpers.DEGREES_TO_RADIANS;
        double sinTheta = Math.sin(theta);
        double cosTheta = Math.cos(theta);

        double xDistAdjusted = pivotToTargetXDist - 
        HardwareConstants.END_EFFECTOR_PIVOT_LENGTH * cosTheta + 
        HardwareConstants.END_EFFECTOR_PIVOT_AXIS_OFFSET * sinTheta;
        
        double topTerm = -TuningConstants.GRAVITY_CONSTANT * xDistAdjusted * xDistAdjusted;
        double bottomTerm = 2 * cosTheta *
        (pivotToTargetYDist * cosTheta - pivotToTargetXDist * sinTheta - 
        HardwareConstants.END_EFFECTOR_PIVOT_AXIS_OFFSET);

        double result = Math.sqrt(topTerm / bottomTerm);
        return result;
    }

    /*
     * gets the difference between the upwards required velocity and
     * the intersecting required velocity
     * 
     * used for finding the intersection by setting this to zero
     */
    private double getResultantAngleIntersection(double theta) {
        theta *= Helpers.DEGREES_TO_RADIANS;
        double sinTheta = Math.sin(theta);
        double cosTheta = Math.cos(theta);

        double difference = 2 * cosTheta * pivotToTargetYDist - sinTheta * pivotToTargetXDist
        - HardwareConstants.END_EFFECTOR_PIVOT_LENGTH * sinTheta * cosTheta - HardwareConstants.END_EFFECTOR_PIVOT_AXIS_OFFSET * (cosTheta * cosTheta + 1);
        return difference;
    }

    /*
     * gets the difference between the max velocity and
     * the intersecting required velocity
     * 
     * used for finding the intersection by setting this to zero
     */

    private double getResultantAngleMaxVelocity(double theta) {
        theta *= Helpers.DEGREES_TO_RADIANS;
        double sinTheta = Math.sin(theta);
        double cosTheta = Math.cos(theta);
        double firstTerm = 2 * cosTheta * (pivotToTargetYDist * cosTheta - pivotToTargetXDist * sinTheta - HardwareConstants.END_EFFECTOR_PIVOT_AXIS_OFFSET);
        double secondTerm = (pivotToTargetXDist - HardwareConstants.END_EFFECTOR_PIVOT_LENGTH * cosTheta + HardwareConstants.END_EFFECTOR_PIVOT_AXIS_OFFSET * sinTheta);
        double sum = firstTerm + TuningConstants.GRAVITY_CONSTANT * secondTerm * secondTerm / (TuningConstants.SHOOTER_MAX_VELOCITY * TuningConstants.SHOOTER_MAX_VELOCITY);
        
        return sum;
    }

    private double getAngleSimple(double xDist, double zDist, double shootingVel)
    {
        double vel = shootingVel * TuningConstants.SHOOTER_VEL_DAMPNER;
        double k = - TuningConstants.GRAVITY_CONSTANT * xDist /  (2.0 * Math.pow(vel, 2)); // constant used throughout
        double tanTheta = ( xDist - Math.sqrt(xDist * xDist - 4 * k * (k - zDist)) ) / (2 * k);
        double theta = Math.abs(Helpers.atand(tanTheta));

        return theta;
    }

}