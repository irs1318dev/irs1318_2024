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
        PositionRetractWrist,
        RetractWrist,
        PositionRetractShoulder,
        RetractShoulder,
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
            this.state = ArmZeroState.PositionRetractWrist;
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
        if (this.state == ArmZeroState.PositionRetractWrist)
        {
            if (currTime >= this.transitionTime + TuningConstants.ARM_WRIST_POWER_TRACKING_DURATION &&
                (this.arm.getWristStalled() ||
                    Math.abs( this.arm.getWristPosition() - TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION) < TuningConstants.ARM_WRIST_ZEROING_POSITION_THRESHOLD ||
                    this.arm.getWristVelocityAverage() < TuningConstants.ARM_WRIST_ZEROING_VELOCITY_THRESHOLD ||
                    currTime >= this.transitionTime + 2.5) )
            {
                // System.out.println("Wrist Stalled: " + this.arm.getWristStalled());
                // System.out.println("Get wrist position: " + this.arm.getWristPosition() + " Get Distance To desired: " + Math.abs( this.arm.getWristPosition() - TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION));
                // System.out.println("Velocity Average: " + this.arm.getWristVelocityAverage());
                // System.out.println("Transition Time: " + this.transitionTime + " Current Time: " + currTime);
                // System.out.println("Moving to percent output");
                this.state = ArmZeroState.RetractWrist;
                this.transitionTime = currTime;
            }
        }
        else if (this.state == ArmZeroState.RetractWrist)
        {
            if (currTime >= this.transitionTime + TuningConstants.ARM_WRIST_POWER_TRACKING_DURATION &&
                (this.arm.getWristStalled() ||
                    this.arm.getWristLimitSwitchStatus() ||
                    this.arm.getWristVelocityAverage() < TuningConstants.ARM_WRIST_ZEROING_VELOCITY_THRESHOLD + 2500 ||
                    currTime >= this.transitionTime + 4.0))
            {
                // System.out.println("Wrist Stalled: " + this.arm.getWristStalled());
                // System.out.println("Get wrist position: " + this.arm.getWristPosition() + " Get Distance To desired: " + Math.abs( this.arm.getWristPosition() - TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION));
                // System.out.println("Velocity Average: " + this.arm.getWristVelocityAverage());
                // System.out.println("Transition Time: " + this.transitionTime + " Current Time: " + currTime);
                // System.out.println("Moving Shoulder");

                this.state = ArmZeroState.PositionRetractShoulder;
                this.transitionTime = currTime;
            }
        }
        else if (this.state == ArmZeroState.PositionRetractShoulder)
        {
            if (currTime >= this.transitionTime + TuningConstants.ARM_WRIST_POWER_TRACKING_DURATION &&
                (this.arm.getShoulderStalled() ||
                    Math.abs(this.arm.getShoulderPosition() - TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION) < TuningConstants.ARM_SHOULDER_ZEROING_POSITION_THRESHOLD ||
                    this.arm.getShoulderVelocityAverage() < TuningConstants.ARM_SHOULDER_ZEROING_VELOCITY_THRESHOLD ||
                    currTime >= this.transitionTime + 1.5))
            {
                this.state = ArmZeroState.RetractShoulder;
                this.transitionTime = currTime;
            }
        }
        else if (this.state == ArmZeroState.RetractShoulder)
        {
            if (currTime >= this.transitionTime + TuningConstants.ARM_WRIST_POWER_TRACKING_DURATION &&
                (this.arm.getShoulderVelocityAverage() < TuningConstants.ARM_SHOULDER_ZEROING_VELOCITY_THRESHOLD ||
                    currTime >= this.transitionTime + 4.0))
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
            case PositionRetractWrist:
                this.setAnalogOperationState(AnalogOperation.ArmWristPower, TuningConstants.ZERO);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPower, TuningConstants.ZERO);
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
                this.setDigitalOperationState(DigitalOperation.ArmStop, false);
                break;

            case RetractWrist:
                this.setAnalogOperationState(AnalogOperation.ArmWristPower, TuningConstants.ARM_WRIST_ZEROING_POWER);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPower, TuningConstants.ZERO);
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
                this.setDigitalOperationState(DigitalOperation.ArmStop, false);
                break;

            case PositionRetractShoulder:
                this.setAnalogOperationState(AnalogOperation.ArmWristPower, TuningConstants.ZERO);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPower, TuningConstants.ZERO);
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
                this.setDigitalOperationState(DigitalOperation.ArmStop, false);
                break;

            case RetractShoulder:
                this.setAnalogOperationState(AnalogOperation.ArmWristPower, TuningConstants.ZERO);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPower, TuningConstants.ARM_SHOULDER_ZEROING_POWER);
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, false);
                this.setDigitalOperationState(DigitalOperation.ArmStop, false);
                break;

            case Reset:
                this.setAnalogOperationState(AnalogOperation.ArmWristPower, TuningConstants.ZERO);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPower, TuningConstants.ZERO);
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
                this.setDigitalOperationState(DigitalOperation.ArmForceReset, true);
                this.setDigitalOperationState(DigitalOperation.ArmStop, true);
                break;

            default:
            case Completed:
                this.setAnalogOperationState(AnalogOperation.ArmWristPower, TuningConstants.ZERO);
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPower, TuningConstants.ZERO);
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
        this.setAnalogOperationState(AnalogOperation.ArmWristPower, TuningConstants.ZERO);
        this.setAnalogOperationState(AnalogOperation.ArmShoulderPower, TuningConstants.ZERO);
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
