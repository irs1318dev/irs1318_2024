package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;


public class StowArmThroughBeamTask extends ControlTaskBase
{

    //private ArmMechanism armMechanism;
    private EndEffectorMechanism endEffectorMechanism;
    private boolean hasCompleted;    
    //private EffectorState endEffectorMechanism.currentEffectorState;
    private ITimer timer;
    private double startTime;
    private double currentTime;
    private double elapsedTime;
    

    @Override
    public void begin()
    {

        //this.armMechanism = this.getInjector().getInstance(ArmMechanism.class);
        this.endEffectorMechanism = this.getInjector().getInstance(EndEffectorMechanism.class);
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.startTime = timer.get();
        
    }

    
    @Override
    public void update()
    {
        this.currentTime = timer.get();
        this.elapsedTime = this.currentTime - this.startTime;
        
        if ((endEffectorMechanism.getEndEffectorState().equals("Intaking")) && (endEffectorMechanism.hasGamePiece())) 
        {
            if (this.elapsedTime >= TuningConstants.STOW_WAIT_TIME) 
            {
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);
                hasCompleted = true;
            }
        }

        else if ((endEffectorMechanism.getEndEffectorState().equals("Shooting")) && (!endEffectorMechanism.hasGamePiece())) 
        {
            if (this.elapsedTime >= TuningConstants.STOW_WAIT_TIME) 
            {
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);
                hasCompleted = true;
            }

        }
    }

    
    @Override
    public void end()
    {
        
    }

    @Override
    public boolean hasCompleted()
    {
        return this.hasCompleted;
    }

}

