package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.FieldConstants;
import frc.robot.HardwareConstants;

import java.util.EnumMap;

import com.google.inject.Injector;

import frc.lib.driver.IControlTask;
import frc.lib.driver.states.AnalogOperationState;
import frc.lib.driver.states.DigitalOperationState;
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
    private EndEffectorMechanism endEffector;
    private OffboardVisionManager visionManager;

    private double desiredVelocity;
    private double desiredAngle;

    private double pivotToTargetXDist;
    private double pivotToTargetYDist;

    private boolean hasCompleted;

    public ShootNoteTask()
    {
        super();

        hasCompleted = false;

    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        this.endEffector = this.getInjector().getInstance(EndEffectorMechanism.class);
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);

        this.setDigitalOperationState(DigitalOperation.VisionEnableAprilTagProcessing, true);
        this.AppendTask(new VisionTurningTask(VisionTurningTask.TurnType.AprilTagCentering));
    }

    @Override
    protected void finishedTask(IControlTask finishedTask)
    {
        super.finishedTask(finishedTask);

        if (finishedTask instanceof VisionTurningTask) {
            //TODO finalize math and offsets correct
            double distToTargetX = visionManager.getAprilTagXOffset() 
            + FieldConstants.APRILTAG_TO_SPEAKER_TARGET_X
            - arm.getXOffset();
            double distToTargetY = visionManager.getAprilTagZOffset()
            + FieldConstants.APRILTAG_TO_SPEAKER_TARGET_Y
            - arm.getZOffset();

            pivotToTargetXDist = distToTargetX;
            pivotToTargetYDist = distToTargetY;
            setDesiredAngleFromXYOffsets(distToTargetX, distToTargetY);
            
            this.AppendTask(ConcurrentTask.AllTasks(
                new SetFlywheelTask(desiredVelocity),
                new SetEndEffectorAngleTask(desiredAngle)
            ));

            this.AppendTask(new KickNoteTask());
        }

        if (finishedTask instanceof KickNoteTask) {
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
        for (int i = 0; i < TuningConstants.ANGLE_FINDING_ITERATIONS; i++) {
            if (getVelocityFromAngleIntersection(midpoint) < 0) {
                rightBound = midpoint;
            }
            else {
                leftBound = midpoint;
            }
            midpoint = (leftBound + rightBound) * 0.5;
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
    private double getVelocityFromAngleIntersection(double theta) {
        theta *= Helpers.DEGREES_TO_RADIANS;
        double sinTheta = Math.sin(theta);
        double cosTheta = Math.cos(theta);

        double difference = 2 * cosTheta * pivotToTargetYDist - sinTheta * pivotToTargetXDist
        - HardwareConstants.END_EFFECTOR_PIVOT_LENGTH * sinTheta * cosTheta - HardwareConstants.END_EFFECTOR_PIVOT_AXIS_OFFSET * (cosTheta * cosTheta + 1);
        return difference;
    }

}