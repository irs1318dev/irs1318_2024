package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that calculates desired angle and velocity to shoot note
 * @author FWJK35 (Calvin)
 */
public class SetEndEffectorAngleTask extends ControlTaskBase
{

    private ArmMechanism armMechanism;

    private double desiredEndEffectorAngle;
    private double currentPos;
    private boolean hasCompleted;

    public SetEndEffectorAngleTask(double desiredEndEffectorAngle)
    {
        this.desiredEndEffectorAngle = desiredEndEffectorAngle;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {

        this.armMechanism = this.getInjector().getInstance(ArmMechanism.class);

        this.setAnalogOperationState(AnalogOperation.ArmAbsWristAngle, this.desiredEndEffectorAngle);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
    }

    /*
     * Update the current task and controls
     */
    @Override
    public void update()
    {
        this.currentPos = this.armMechanism.getAbsoluteAngleOfShot();
        if(Helpers.RoughEquals(this.currentPos, this.desiredEndEffectorAngle, TuningConstants.ARM_WRIST_GOAL_THRESHOLD))
        {
            this.hasCompleted = true;
        }

        this.setAnalogOperationState(AnalogOperation.ArmAbsWristAngle, this.desiredEndEffectorAngle);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ArmAbsWristAngle, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.hasCompleted;
    }

}

