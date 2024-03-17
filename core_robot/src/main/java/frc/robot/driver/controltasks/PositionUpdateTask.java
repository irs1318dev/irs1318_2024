package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class PositionUpdateTask extends UpdateCycleTask
{
    public PositionUpdateTask()
    {
        super(5);
    }

    @Override
    public void update()
    {
        if (this.currentUpdates == 0)
        {
            this.setDigitalOperationState(DigitalOperation.PositionSwapFieldOrientation, true);
        }
        else
        {
            this.setDigitalOperationState(DigitalOperation.PositionSwapFieldOrientation, false);
        }

        this.setDigitalOperationState(DigitalOperation.DriveTrainKeepThisOrientation, true);
        super.update();
    }

    public void end()
    {
        super.end();
        this.setDigitalOperationState(DigitalOperation.PositionSwapFieldOrientation, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainKeepThisOrientation, false);
    }
}
