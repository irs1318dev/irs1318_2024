package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ArmIKLimitingTask extends CompositeOperationTask {
    
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.ArmEnableIKLimiting,
            DigitalOperation.ArmDisableIKLimiting,
        };

    public ArmIKLimitingTask(boolean enable)
    {
        super(
            enable ? DigitalOperation.ArmEnableIKLimiting : DigitalOperation.ArmDisableIKLimiting,
            ArmIKLimitingTask.possibleOperations);
    }

    
}
