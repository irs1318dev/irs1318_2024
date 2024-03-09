package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.FieldConstants;
import frc.robot.HardwareConstants;

import frc.lib.driver.IControlTask;
import frc.lib.helpers.Helpers;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that calculates desired angle and velocity to shoot note
 * @author FWJK35 (Calvin)
 */
public class ShootNoteTask extends DecisionSequentialTask
{
    private ArmMechanism arm;
    private OffboardVisionManager visionManager;

    private boolean useMaxVelocity;

    private double desiredVelocity;
    private double desiredAngle;

    private double pivotToTargetXDist;
    private double pivotToTargetYDist;

    private boolean hasCompleted;

    public ShootNoteTask()
    {
        this(false);
    }

    public ShootNoteTask(boolean useMaxVelocity)
    {
        super();

        this.useMaxVelocity = useMaxVelocity;

        hasCompleted = false;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);

        this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, true);
        this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagFront, false);
        this.setDigitalOperationState(DigitalOperation.VisionFindAnyAprilTagRear, false);
        this.setDigitalOperationState(DigitalOperation.VisionFindAnyAprilTagFront, false);
        this.AppendTask(new VisionTurningTask(VisionTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear));
    }

    @Override
    protected void finishedTask(IControlTask finishedTask)
    {
        super.finishedTask(finishedTask);

        if (finishedTask instanceof VisionTurningTask) {
            double distToTargetX = visionManager.getAprilTagXOffset() 
            + FieldConstants.APRILTAG_TO_SPEAKER_TARGET_X
            + arm.getWristJointAbsPosition()[0];
            double distToTargetY = 57.5//visionManager.getAprilTagZOffset()
            + FieldConstants.APRILTAG_TO_SPEAKER_TARGET_Y
            - arm.getWristJointAbsPosition()[1];

            pivotToTargetXDist = distToTargetX;
            pivotToTargetYDist = distToTargetY;
            setDesiredAngleFromXYOffsets(distToTargetX, distToTargetY);
            
// System.out.println("X: " -);

            this.AppendTask(ConcurrentTask.AllTasks(
<<<<<<< Updated upstream
                // new ShooterSpinTask(desiredVelocity * TuningConstants.SHOOTER_DRAG_COMPENSATION_MULTIPLIER),
                new SetEndEffectorAngleTask(desiredAngle)
=======
                // new ShooterSpinTask(this.desiredVelocity / (Math.PI * 2 * HardwareConstants.SHOOTER_FLYWHEEL_RADIUS) * 60 * TuningConstants.SHOOTER_DRAG_COMPENSATION_MULTIPLIER),
                new SetEndEffectorAngleTask(this.desiredAngle)
>>>>>>> Stashed changes
            ));

            // this.AppendTask(SequentialTask.Sequence(
                // new IntakeControlTask(false, TuningConstants.KICK_OUTTAKE_TIME),
                // new FeedRingTask(true, TuningConstants.KICK_INTAKE_TIME)
            // ));
        }

        if (finishedTask instanceof SequentialTask) {
            hasCompleted = true;
        }
    }

    /*
     * Update the current task and controls
     */
    @Override
    public void update()
    {
        
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {

    }

    @Override
    public boolean hasCompleted()
    {
        return hasCompleted;
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

        //if it should fire with the max velocity, calculate that angle
        if (useMaxVelocity) {
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

}