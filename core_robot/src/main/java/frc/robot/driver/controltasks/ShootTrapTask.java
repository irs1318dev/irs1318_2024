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
    double xOffset;
    double yOffset;
    DigitalOperation visionOperation;

    
    public ShootTrapTask (double xOffset, double yOffset, DigitalOperation visionOperation)
    {
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.visionOperation = visionOperation;
    }


    @Override
    public void begin()
    {
        hasCompleted = false;
        this.AppendTask(
            ConcurrentTask.AllTasks(
                new ShooterSpinTask(TuningConstants.TRAP_SHOT_FLYWHEEL_VELOCITY),
                SequentialTask.Sequence(
                    new VisionApproachAprilTagTask(this.xOffset, this.yOffset, this.visionOperation),
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_TRAP_SHOOT, TuningConstants.ARM_WRIST_TRAP_SHOOT),
                    new FeedRingTask(true))
        ));

        hasCompleted();
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
