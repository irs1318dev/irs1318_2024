package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;

/**
 * Abstract class defining a task that lasts only for a certain number of update cycles.
 * 
 */
public class ArmShoulderPositionTask extends UpdateCycleTask
{
    private final double position;

    /**
     * Initializes a new ArmShoulderPositionTask
     * @param position desired
     */
    public ArmShoulderPositionTask(double position)
    {
        super(1);

        this.position = position;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, this.position);
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        super.update();

        this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, this.position);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
    }
}
