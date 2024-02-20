package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakeControlTask extends CompositeOperationTask{

    private static final DigitalOperation[] possibleOperations = 
    {
        DigitalOperation.ShooterFeedRing,
        DigitalOperation.IntakeOut
    };

    public IntakeControlTask(boolean intakeIn)
    {
        super(
            intakeIn ? DigitalOperation.ShooterFeedRing : DigitalOperation.IntakeOut,
            IntakeControlTask.possibleOperations,
            true);
    }

    public IntakeControlTask(boolean intakeIn, double timeout)
    {
        super(
            intakeIn ? DigitalOperation.ShooterFeedRing : DigitalOperation.IntakeOut,
            IntakeControlTask.possibleOperations,
            timeout);
    }
}
