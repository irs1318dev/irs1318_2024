package frc.robot.driver.controltasks;

import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that calculates desired angle and velocity to shoot note
 * @author FWJK35 (Calvin)
 */
public class SetEndEffectorAngleTask extends ControlTaskBase
{

    private final ArmMechanism armMechanism;

    private final double desiredEndEffectorAngle;

    public SetEndEffectorAngleTask(double desiredEndEffectorAngle)
    {
        this.armMechanism = this.getInjector().getInstance(ArmMechanism.class);

        this.desiredEndEffectorAngle = desiredEndEffectorAngle;
    }



    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, desiredEndEffectorAngle);
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
        //TODO implement arm mechanism has reached target absolute wrist goal
        return true;
    }

}