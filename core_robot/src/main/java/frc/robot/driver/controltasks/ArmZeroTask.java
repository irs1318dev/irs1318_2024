package frc.robot.driver.controltasks;

import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that returns the arm to its default, fully-retracted position
 * 
 */
public class ArmZeroTask extends ControlTaskBase
{
    // Note: we retract wrist before shoulder to make sure that we don't crunch the intake
    private enum ArmZeroState
    {
        RetractWrist,
        Stop,
        Reset,
        Completed;
    }

    private ArmMechanism arm;
    private ITimer timer;

    private ArmZeroState state;
    private double transitionTime;

    public ArmZeroTask()
    {
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        this.timer = this.getInjector().getInstance(ITimer.class);

        if (this.arm.getInSimpleMode())
        {
            this.state = ArmZeroState.Completed;
        }
        else
        {
            this.state = ArmZeroState.RetractWrist;
            this.transitionTime = timer.get();
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        double currTime = this.timer.get();
        if (this.state == ArmZeroState.RetractWrist)
        {
            if (currTime >= this.transitionTime + TuningConstants.ARM_WRIST_POWER_TRACKING_DURATION &&
                (this.arm.getWristStalled() || currTime >= this.transitionTime + 1.5))
            {
                this.state = ArmZeroState.Stop;
                this.transitionTime = currTime;
            }
        }
        else if (this.state == ArmZeroState.Stop)
        {
            if (currTime >= this.transitionTime + 2.5)
            {
                this.state = ArmZeroState.Reset;
            }
        }
        else if (this.state == ArmZeroState.Reset)
        {
            this.state = ArmZeroState.Completed;
        }

        switch (this.state)
        {
            case RetractWrist:
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.ARM_WRIST_POSITION_RESET);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
                this.setDigitalOperationState(DigitalOperation.ArmStop, false);
                break;

            case Stop:
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
                this.setDigitalOperationState(DigitalOperation.ArmStop, true);
                break;

            case Reset:
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, true);
                this.setDigitalOperationState(DigitalOperation.ArmStop, true);
                break;

            default:
            case Completed:
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
                this.setDigitalOperationState(DigitalOperation.ArmStop, false);
                break;
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
        this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
        this.setDigitalOperationState(DigitalOperation.ArmStop, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.state == ArmZeroState.Completed;
    }
}
