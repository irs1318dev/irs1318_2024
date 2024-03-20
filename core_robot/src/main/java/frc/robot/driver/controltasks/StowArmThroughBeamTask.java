package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;


public class StowArmThroughBeamTask extends ControlTaskBase
{

    private ArmMechanism armMechanism;
    private EndEffectorMechanism endEffectorMechanism;
    private boolean hasCompleted;
    private ITimer timer;
    private double startTime;
    private double currentTime;
    

    @Override
    public void begin()
    {

        this.armMechanism = this.getInjector().getInstance(ArmMechanism.class);
        this.endEffectorMechanism = this.getInjector().getInstance(EndEffectorMechanism.class);
        this.timer = this.getInjector().getInstance(ITimer.class);
        
    }

    
    @Override
    public void update()
    {
        this.currentTime = timer.get();
        
        if ((endEffectorMechanism.getEndEffectorState().equals("Intaking")) &&
                endEffectorMechanism.hasGamePiece() &&
                Helpers.RoughEquals(armMechanism.getShoulderPosition(), TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_SHOULDER_AUTO_STOW_THRESHOLD) &&
                Helpers.RoughEquals(armMechanism.getWristPosition(), TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP, TuningConstants.ARM_WRIST_AUTO_STOW_THRESHOLD)) 
        {
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.ARM_WRIST_POSITION_QUICK_TUCK);
            hasCompleted = true;
        }

        else if ((endEffectorMechanism.getEndEffectorState().equals("Shooting")) &&
                !endEffectorMechanism.hasGamePiece() &&
                Helpers.RoughEquals(armMechanism.getShoulderPosition(), TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_SHOULDER_AUTO_STOW_THRESHOLD) &&
                Helpers.RoughEquals(armMechanism.getWristPosition(), TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT, TuningConstants.ARM_WRIST_AUTO_STOW_THRESHOLD)) 
        {
            this.startTime = this.currentTime;
            if (this.currentTime - this.startTime >= TuningConstants.STOW_WAIT_TIME) 
            {
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.ARM_WRIST_POSITION_QUICK_TUCK);
                hasCompleted = true;
            }

        }
    }

    
    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.hasCompleted;
    }

}

