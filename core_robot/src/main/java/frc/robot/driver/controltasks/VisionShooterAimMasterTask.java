public class VisionShooterAimMasterTask extends ControlTaskBase
{
    public static IControlTask createShootMacroTask(boolean continuous)
    {
        return 
            SequentialTask.Sequence(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                ConcurrentTask.AllTasks(
                    new SpeakerAbsoluteOrientationTask(),
                    new ShootVisionMasterTask()));          
    }

    private OffboardVisionManager vision;
    private ArmMechanism arm;
    private EndEffectorMechanism endEffector;
    private SDSDriveTrainMechanism driveTrain;

    private final LinearInterpolator angleLinterp;
    private final LinearInterpolator velocityLinterp;
    private double distanceToSpeaker;
    private double farFlywheelVelocity;
    private double nearFlywheelVelocity;
    private double wristAngle;
    private int noTargetCount;
    private boolean shouldCancel;
    
    private enum State
    {
        FindSpeakerAprilTag,
        SetWristAndVelocity,
        RumbleDriver,
        Completed
    }

    private State currentState;
    
    public ShootVisionMasterTask()
    {
        super();
        this.angleLinterp = new LinearInterpolator(TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES, TuningConstants.SHOOT_VISION_SAMPLE_ANGLES);
        this.velocityLinterp = new LinearInterpolator(TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES, TuningConstants.SHOOT_VISION_SAMPLE_VELOCITIES);
    }

    @Override 
    public void begin()
    {
        this.vision = this.getInjector().getInstance(OffboardVisionManager.class);
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        this.endEffector = this.getInjector().getInstance(EndEffectorMechanism.class);
        this.driveTrain = this.getInjector().getInstance(SDSDriveTrainMechanism.class);

        this.farFlywheelVelocity = TuningConstants.MAGIC_NULL_VALUE;
        this.nearFlywheelVelocity = TuningConstants.MAGIC_NULL_VALUE;
    }

    @Override 
    public void update()
    {
        if (this.currentState == State.FindSpeakerAprilTag) 
        {
            distancetoSpeaker = Math.sqrt(
                (Math.pow(vision.getAbsX() - TuningConstants.CENTER_TO_SPEAKER_X_DISTANCE), 2) +
                (Math.pow(vision.getAbsY() - TuningConstants.CENTER_TO_SPEAKER_Y_DISTANCE), 2));
            
            if (distanceToSpeaker == null) 
            {
                this.noTargetCount++;
                if (this.noTargetCount > TuningConstants.SHOOT_VISION_APRILTAG_NOT_FOUND_THRESHOLD)
                {
                    this.shouldCancel = true;
                }
            }
            else 
            {
                this.wristAngle = this.angleLinterp.sample(distanceToSpeaker);
                this.farFlywheelVelocity = this.velocityLinterp.sample(distanceToSpeaker);
                this.nearFlywheelVelocity = this.velocityLinterp.sample(distanceToSpeaker);
                this.currentState = State.SetWristAndVelocity;
            }
        }

        if (this.currentState == State.SetWristAndVelocity)
        {
            this.noTargetCount = 0;
            if (Helpers.RoughEquals(this.arm.getWristPosition(), this.wristAngle, TuningConstants.SHOOT_VISION_WRIST_ACCURACY_THRESHOLD))
            {
                this.currentState = State.FindSpeakerAprilTag;  
            }
            if ((Helpers.RoughEquals(driveTrain.getForwardFieldVelocity, 0, TuningConstants.ACCEPTABLE_NOT_MOVING_RANGE)) && 
            (Helpers.RoughEquals(driveTrain.getLeftFieldVelocity, 0, TuningConstants.ACCEPTABLE_NOT_MOVING_RANGE))) 
            {
                this.currentState = State.RumbleDriver;
            }
        }

        if(this.currentState == State.RumbleDriver)
        {
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.5);
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.5);
        }
        
        switch (this.currentState)
        {
            case FindSpeakerAprilTag:
                this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, true);       
                this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, this.nearFlywheelVelocity);
                this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, this.farFlywheelVelocity);
                break;

            case SetWristAndVelocity:
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristAngle); 
                this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, this.nearFlywheelVelocity);
                this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, this.farFlywheelVelocity);
                this.setDigitalOperationState(DigitalOperation.ForceLightDriverRumble, true);
                break;

            default:
            case Completed:
                break;
        }
    }

    @Override 
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, false);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean shouldCancel()
    {
        return this.shouldCancel;
    }

    @Override
    public boolean hasCompleted()
    {
        return this.currentState == State.Completed;
    }
}
