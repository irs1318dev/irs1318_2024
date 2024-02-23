package frc.robot.driver.controltasks;

import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ClimberMechanism;

public class ClimberWinchTask extends ControlTaskBase
{
    private ITimer timer; 

    public enum WinchState
    {
        Extended,
        Retracted,
        Extending,
        Retracting
    }

    private double startTime;
    private double currentTime;
    private double timeSinceStart;

    private WinchState goalState;
    private WinchState currentState = null;

    public ClimberWinchTask()
    {
        this(WinchState.Retracted);
    }

    public ClimberWinchTask(WinchState desiredState)
    {
        this.goalState = desiredState;
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
        // update timer and timeSinceStart every tick
        this.currentTime = this.timer.get();
        this.timeSinceStart = currentTime - startTime;

        if (this.goalState == WinchState.Extended)
        {
            if (this.timeSinceStart < TuningConstants.CLIMBER_FULL_EXTEND_TIME)
            {
                this.currentState = WinchState.Extending;
                this.setDigitalOperationState(DigitalOperation.ClimberWinchUp, true);
            }
            else
            {
                this.currentState = WinchState.Extended;
                this.setDigitalOperationState(DigitalOperation.ClimberWinchDown, false);
            }
        }

        if (this.goalState == WinchState.Retracted)
        {
            if (ClimberMechanism.getClimberDown() == false) //if limit switch hasn't been tripped yet
            {
                this.currentState = WinchState.Retracting;
                this.setDigitalOperationState(DigitalOperation.ClimberWinchDown, true);
            }
            else //if limit switch is true
            {
                this.currentState = WinchState.Retracted;
                this.setDigitalOperationState(DigitalOperation.ClimberWinchDown, false);
            }
        }
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.ClimberWinchDown, false);
    }

    @Override
    public boolean hasCompleted()
    {
        //limitswitch?
        return (this.currentState == WinchState.Retracted || this.currentState == WinchState.Extended);
    }
}
