package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.HardwareConstants;
import frc.lib.helpers.Helpers;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that calculates desired angle and velocity to shoot note
 * @author FWJK35 (Calvin)
 */
public class ShootNoteTask extends ControlTaskBase
{
    private final ArmMechanism arm;
    private final EndEffectorMechanism endEffector;

    private final double desiredVelocity;
    private final double desiredAngle;

    private final double pivotToTargetXDist;
    private final double pivotToTargetYDist;

    public ShootNoteTask()
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        this.endEffector = this.getInjector().getInstance(EndEffectorMechanism.class);
        
        //TODO get the correct X and Y offsets, based on arm length and shoulder rotation
        //get current X offset
        //get current Y offset
        pivotToTargetXDist = 240; //inches
        pivotToTargetYDist = 120; //inches

        //find the intersection of upwards velocity and hitting the target
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
        desiredAngle = midpoint + TuningConstants.SHOOTER_FINAL_ANGLE_OFFSET;
        desiredVelocity = getVelocityFromAngleTarget(desiredAngle);

    }



    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        //TODO set the wrist angle and flywheel velocity

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
        return true;
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
     * gets the required velocity to pass the x-coord of the target 
     * with an upwards velocity with a given angle
     */
    private double getVelocityFromAngleUpwards(double theta) {
        theta *= Helpers.DEGREES_TO_RADIANS;

        //x distance to target, end effector offsets included
        double xDistAdjusted = pivotToTargetXDist - 
        HardwareConstants.END_EFFECTOR_PIVOT_LENGTH * Math.cos(theta) + 
        HardwareConstants.END_EFFECTOR_PIVOT_AXIS_OFFSET * Math.sin(theta);
        
        double topTerm = TuningConstants.GRAVITY_CONSTANT * xDistAdjusted;
        double bottomTerm = Math.sin(theta) * Math.cos(theta);

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