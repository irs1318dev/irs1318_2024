package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ArmSlowModeTask extends ControlTaskBase
{
    public ArmSlowModeTask()
    {
    }

    @Override
    public void begin()
    {
        this.setDigitalOperationState(DigitalOperation.ArmSlowMode, true);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.ArmSlowMode, true);
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.ArmSlowMode, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
