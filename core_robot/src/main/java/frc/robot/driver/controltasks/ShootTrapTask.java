package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.FieldConstants;
import frc.robot.HardwareConstants;

import frc.lib.driver.IControlTask;
import frc.lib.helpers.Helpers;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;


public class ShootTrapTask extends DecisionSequentialTask
{
    private boolean hasCompleted;

    
    public ShootTrapTask ()
    {
    }


    @Override
    public void begin()
    {
        hasCompleted = false;
        this.AppendTask(new ApproachAprilTagTask(TRAP_APRILTAG_TO_ROBOT_X, TRAP_APRILTAG_TO_ROBOT_Y, ));
    }

    @Override
    protected void finishedTask(IControlTask finishedTask)
    {
        super.finishedTask(finishedTask);

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
    
}
