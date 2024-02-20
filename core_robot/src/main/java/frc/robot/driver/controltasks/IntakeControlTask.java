package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakeControlTask extends CompositeOperationTask{

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
    }

    public IntakeControlTask(boolean intakeIn, double timeout)
    {
        super(
            intakeIn ? DigitalOperation.IntakeIn : DigitalOperation.IntakeOut,
            IntakeControlTask.possibleOperations,
            timeout);
    }
}
