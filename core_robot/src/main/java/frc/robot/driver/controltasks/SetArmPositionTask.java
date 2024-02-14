public class SetArmPositionTask extends ControlTaskBase 
{
    private ArmMechanism arm;
    private double desiredLowerArmPosition;
    private double desiredLowerWristPosition,;
    private double desiredUpperArmPosition;
    private double desiredUpperWristPosition;
    private double desiredArmGoal;
    private double desiredWristGoal;
    public enum arm
    {
        desiredLowerArmWristIntermediate,
        desiredUpperArmWristIntermediate,
        desiredGoal,
        Completed,
    }
    private Arm armState;

    public SetArmPositionTask(double desiredLowerArmPosition, 
        double desiredLowerWristPosition, 
        double desiredUpperArmPosition,
        double desiredUpperWristPosition,
        double desiredArmGoal,
        double desiredWristGoal,
    )
    {
        this.desiredLowerArmPosition = desiredLowerArmPosition;
        this.desiredLowerWristPosition = desiredLowerWristPosition;
        this.desiredUpperArmPosition = desiredUpperArmPosition;
        this.desiredUpperWristPosition = desiredUpperWristPosition;
        this.desiredArmGoal = desiredArmGoal;
        this.desiredWristGoal = desiredWristGoal;
        armState = desiredLowerArmWristIntermediate;
    }

    @Override
    public void begin() 
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        
    }

    @Overide 
    public void update()
    {
        if (this.armState == desiredLowerArmWristIntermediate)
        {
            this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, desiredLowerArmPosition);
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, desiredLowerWristPosition);
            this.armState = desiredUpperArmWristIntermediate
        }
        
        else if (this.armState == desiredUpperArmWristIntermediate)
        {
            this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, desiredUpperArmPosition);
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, desiredUpperWristPosition);
            this.state = desiredGoal;
        }

        else if (this.armState == desiredGoal)
        {
            this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, desiredArmGoal);
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, desiredWristGoal); 
            this.armState = hasCompleted;
        }
        
        


        
    }

    @Overide
    public void end() 
    {
        this.setAnalogOperationState(AnalogOperation.ArmShoulderPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE); 
    }

    @Overide
    public void hasCompleted()
    {
        this.armState = hasCompleted;
    }
}