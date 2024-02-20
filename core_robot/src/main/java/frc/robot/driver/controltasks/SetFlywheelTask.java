package frc.robot.driver.controltasks;

import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that calculates desired angle and velocity to shoot note
 * @author FWJK35 (Calvin)
 */
public class SetFlywheelTask extends ControlTaskBase
{
    private final double desiredExitVelocity;
    private final double desiredFlywheelVelocity;

    private EndEffectorMechanism endEffector;

    public SetFlywheelTask(double desiredExitVelocity)
    {
        this.desiredExitVelocity = desiredExitVelocity;
        this.desiredFlywheelVelocity = this.getRPMfromDesiredSpeed(this.desiredExitVelocity);
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.endEffector = this.getInjector().getInstance(EndEffectorMechanism.class);

        this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, this.desiredFlywheelVelocity);
        this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, this.desiredFlywheelVelocity);
    }

    /*
     * Update the current task and controls
     */
    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, this.desiredFlywheelVelocity);
        this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, this.desiredFlywheelVelocity);
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
        return this.endEffector.isFlywheelSpunUp();
    }

    private double getRPMfromDesiredSpeed(double desiredExitVelocity)
    {
        //TODO math to get correct rotations per minute from inches per second
        return desiredExitVelocity * 60 / (2 * Math.PI * HardwareConstants.SHOOTER_FLYWHEEL_RADIUS) ;
    }
}