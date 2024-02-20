package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class FeedRingTask extends CompositeOperationTask{

    private static final DigitalOperation[] possibleOperations = 
    {
        DigitalOperation.ShooterFeedRing,
        DigitalOperation.IntakeForceOnAndIntakeIn,
    };

    public FeedRingTask(boolean waitForSpunUp)
    {
        super(
            waitForSpunUp ? DigitalOperation.ShooterFeedRing : DigitalOperation.IntakeForceOnAndIntakeIn,
            FeedRingTask.possibleOperations,
            true);
    }
}