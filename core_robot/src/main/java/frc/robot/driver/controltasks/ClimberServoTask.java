package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ClimberServoTask extends CompositeOperationTask {
    
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.ClimberServoDown,
            DigitalOperation.ClimberServoUp,
        };

    public ClimberServoTask(boolean servoDown)
    {
        super(
            servoDown ? DigitalOperation.ClimberServoDown : DigitalOperation.ClimberServoUp,
            ClimberServoTask.possibleOperations,
            true);
    }
}
