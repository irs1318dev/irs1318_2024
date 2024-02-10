package frc.robot.driver.controltasks;

import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that calculates desired angle and velocity to shoot note
 * @author FWJK35 (Calvin)
 */
public class KickNoteTask extends ControlTaskBase
{

    private final EndEffectorMechanism endEffector;

    public KickNoteTask()
    {
        this.endEffector = this.getInjector().getInstance(EndEffectorMechanism.class);
    }



    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.setDigitalOperationState(DigitalOperation.ShooterFeedRing, true);
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
        return endEffector.isFlywheelSpunUp();
    }

    private double getRPMfromDesiredSpeed(double desiredExitVelocity)
    {
        //TODO math to get correct rotations per minute from inches per second
        return desiredExitVelocity;
    }

}