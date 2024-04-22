package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

/**
 * Task that sets the Arm to the desired position using MM
 *
 */
public class ArmPositionTask extends ControlTaskBase
{
    private enum ArmState
    {
        StowWrist,
        MoveToInitialUniversal,
        MoveToFinalUniversal,
        TargetWrist,
        TargetShoulder,
        Completed
    }

    private final double shoulderGoalPosition;
    private final double wristGoalPosition;
    private ArmMechanism arm;

    private ArmState currentArmState;

    boolean curWristToStowed;
    boolean curMoveToLowerUniv;
    boolean goalMoveToLowerUniv;

    public ArmPositionTask(double shoulderPos, double wristPos)
    {
        this.shoulderGoalPosition = shoulderPos;
        this.wristGoalPosition = wristPos;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);

        double curShoulderAngle = this.arm.getShoulderPosition();

        this.curMoveToLowerUniv = Math.abs(curShoulderAngle - TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL)
            < Math.abs(curShoulderAngle - TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL);

        this.goalMoveToLowerUniv = Math.abs(shoulderGoalPosition - TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL)
            < Math.abs(wristGoalPosition - TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL);

        this.curWristToStowed = !(this.goalMoveToLowerUniv == this.curMoveToLowerUniv);

        if (curWristToStowed)
        {
            currentArmState = ArmState.StowWrist;
        }
        else
        {
            currentArmState = ArmState.MoveToInitialUniversal;
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        if (this.currentArmState == ArmState.StowWrist)
        {
            if (Helpers.RoughEquals(this.arm.getWristPosition(), TuningConstants.ARM_WRIST_POSITION_STOWED, TuningConstants.ARM_WRIST_GOAL_THRESHOLD) || this.arm.getStuckInPosition())
            {
                this.currentArmState = ArmState.MoveToInitialUniversal;
            }
            else
            {
                this.currentArmState = ArmState.StowWrist;
            }
        }
        else if (this.currentArmState == ArmState.MoveToInitialUniversal)
        {
            if (Helpers.RoughEquals(this.arm.getWristPosition(), (this.curMoveToLowerUniv ? TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL : TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL), TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD))
            {
                this.currentArmState = ArmState.MoveToFinalUniversal;
            }
            else
            {
                this.currentArmState = ArmState.MoveToInitialUniversal;
            }
        }
        else if (this.currentArmState == ArmState.MoveToFinalUniversal)
        {
            if (Helpers.RoughEquals(this.arm.getWristPosition(), (this.goalMoveToLowerUniv ? TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL : TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL), TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD))
            {
                this.currentArmState = ArmState.TargetWrist;
            }
            else
            {
                this.currentArmState = ArmState.MoveToFinalUniversal;
            }
        }
        else if (this.currentArmState == ArmState.TargetWrist)
        {
            if (Helpers.RoughEquals(this.arm.getWristPosition(), wristGoalPosition, TuningConstants.ARM_WRIST_GOAL_THRESHOLD))
            {
                this.currentArmState = ArmState.TargetShoulder;
            }
            else
            {
                this.currentArmState = ArmState.TargetWrist;
            }
        }
        else if (this.currentArmState == ArmState.TargetShoulder)
        {
            if (Helpers.RoughEquals(this.arm.getWristPosition(), shoulderGoalPosition, TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD))
            {
                this.currentArmState = ArmState.Completed;
            }
            else
            {
                this.currentArmState = ArmState.TargetShoulder;
            }
        }

        switch (this.currentArmState)
        {
            case StowWrist:
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.ARM_WRIST_POSITION_STOWED);
                break;

            case MoveToInitialUniversal:
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, (this.curMoveToLowerUniv ? TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL : TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL));
                break;

            case MoveToFinalUniversal:
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, (this.goalMoveToLowerUniv ? TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL : TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL));
                break;

            case TargetWrist:
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristGoalPosition);
                break;

            case TargetShoulder:
                this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, this.shoulderGoalPosition);
                break;

            case Completed:
                break;
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.currentArmState == ArmState.Completed;
    }
}