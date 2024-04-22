package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.ArmMechanism;

/**
 * Abstract class defining a task that lasts only for a certain number of update cycles.
 * 
 */
public class ArmWristPositionTask extends UpdateCycleTask
{
    private final double position;
    private final boolean waitForPosition;
    private ArmMechanism arm;

    /**
     * Initializes a new ArmWristPositionTask
     * @param position desired
     */
    public ArmWristPositionTask(double position)
    {
        this(position, false);
    }

    /**
     * Initializes a new ArmWristPositionTask
     * @param position desired
     */
    public ArmWristPositionTask(double position, boolean waitForPosition)
    {
        super(1);

        this.position = position;
        this.waitForPosition = waitForPosition;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        if (this.waitForPosition)
        {
            this.arm = this.getInjector().getInstance(ArmMechanism.class);
        }

        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.position);
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        super.update();

        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.position);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.waitForPosition)
        {
            double armWristPosition = this.arm.getWristPosition();
            return Helpers.RoughEquals(this.position, armWristPosition, TuningConstants.ARM_WRIST_GOAL_THRESHOLD);
        }

        return super.hasCompleted();
    }
}
