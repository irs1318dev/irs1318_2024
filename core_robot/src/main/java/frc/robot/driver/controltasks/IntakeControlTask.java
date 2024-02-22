package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.EndEffectorMechanism;

public class IntakeControlTask extends CompositeOperationTask{

    private boolean intakeIn;
    private EndEffectorMechanism endEffector;

    private static final DigitalOperation[] possibleOperations = 
    {
        DigitalOperation.IntakeIn,
        DigitalOperation.IntakeOut
    };

    public IntakeControlTask(boolean intakeIn)
    {
        super(
            intakeIn ? DigitalOperation.IntakeIn : DigitalOperation.IntakeOut,
            IntakeControlTask.possibleOperations,
            true);
        this.intakeIn = intakeIn;
    }

    public IntakeControlTask(boolean intakeIn, double timeout)
    {
        super(
            intakeIn ? DigitalOperation.IntakeIn : DigitalOperation.IntakeOut,
            IntakeControlTask.possibleOperations,
            timeout);
        this.intakeIn = intakeIn;
    }

    @Override
    public void begin() {
        super.begin();
        this.endEffector = this.getInjector().getInstance(EndEffectorMechanism.class);
    }

    @Override
    public boolean hasCompleted() {
        if (this.intakeIn && this.endEffector.hasGamePiece()) {
            return true;
        }
        return super.hasCompleted();
    }
}
