package frc.robot.driver.controltasks;

import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.EndEffectorMechanism;

public class ForwardsUntilNoteTask extends ControlTaskBase {

    private boolean throughBeamBroken = false;
    private EndEffectorMechanism endEffector;

    @Override
    public void begin()
    {
        //begin moving forwards and intaking
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.5);
        this.setDigitalOperationState(DigitalOperation.IntakeIn, true);
        this.throughBeamBroken = endEffector.hasGamePiece();
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.5);
        this.setDigitalOperationState(DigitalOperation.IntakeIn, true);
    }

    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0);
        this.setDigitalOperationState(DigitalOperation.IntakeIn, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return throughBeamBroken;
    }

    //concurrent any task
    //drivetrain forwards 
    //intaking <- until through beam is broken.
    
}
