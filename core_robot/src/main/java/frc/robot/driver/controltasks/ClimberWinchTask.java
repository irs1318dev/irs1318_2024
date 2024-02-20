package frc.robot.driver.controltasks;

import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;

public class ClimberWinchTask extends ControlTaskBase{

    private ITimer timer; 
    public enum WinchState {
        Extended,
        Retracted,
        Extending,
        Retracting
    }

    private double startTime;
    private double currentTime;
    private double timeSinceStart;
    WinchState goalState;;
    WinchState currentState = null;

    public ClimberWinchTask(){
        this(WinchState.Retracted);
    }

    public ClimberWinchTask(WinchState desiredState){
        goalState = desiredState;
    }

    
    @Override
    public void begin()
    {
        // initialize timer
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.startTime = this.timer.get();
    }

    @Override
    public void update()
    {
        //update timer and timeSinceStart every tick
        this.currentTime = this.timer.get();
        this.timeSinceStart = currentTime - startTime;


        if (goalState == WinchState.Extended) {

            if (this.currentTime < TuningConstants.CLIMBER_FULL_EXTEND_TIME) {
                this.currentState = WinchState.Extending;
                this.setDigitalOperationState(DigitalOperation.ClimberWinchUp, true);
            }
            
            else {
                this.currentState = WinchState.Extended;
                this.setDigitalOperationState(DigitalOperation.ClimberWinchDown, false);
            }
            
        }
        

        if (goalState == WinchState.Retracted) {

            if (this.currentTime <  TuningConstants.CLIMBER_FULL_RETRACT_TIME) {
                this.currentState = WinchState.Retracting;
                this.setDigitalOperationState(DigitalOperation.ClimberWinchDown, true);
            }

            else {
                this.currentState = WinchState.Retracted;
                this.setDigitalOperationState(DigitalOperation.ClimberWinchDown, false);
            }
            
        }
    }

    @Override
    public void end()
    {
        // don't forget to add a servo task at the end of the climb
    }

    @Override
    public boolean hasCompleted()
    {
        return (this.currentState == WinchState.Retracted||currentState == WinchState.Extended);
    }
    
}
