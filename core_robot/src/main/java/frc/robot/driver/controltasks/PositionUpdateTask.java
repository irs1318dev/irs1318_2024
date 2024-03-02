package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class PositionUpdateTask extends UpdateCycleTask
{
    public PositionUpdateTask()
    {
        super(1);
    }

    @Override
    public void update()
    {
        super.update();
        this.setDigitalOperationState(DigitalOperation.PositionSwapFieldOrientation, true);
    }

    public void end()
    {
        super.end();
        this.setDigitalOperationState(DigitalOperation.PositionSwapFieldOrientation, false);
    }
}
