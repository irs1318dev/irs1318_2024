package frc.robot.driver.controltasks;
import frc.lib.driver.IControlTask;
import frc.lib.helpers.Helpers;
import frc.lib.helpers.LinearInterpolator;
import frc.lib.robotprovider.IRobotProvider;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ArmMechanism;
import frc.robot.mechanisms.DriverFeedbackManager;
import frc.robot.mechanisms.EndEffectorMechanism;
import frc.robot.mechanisms.OffboardVisionManager;
import frc.robot.mechanisms.SDSDriveTrainMechanism;

public class VisionShooterAimMasterTask extends ControlTaskBase
{
    public static IControlTask createShootMacroTask(boolean continuous)
    {
        return 
            SequentialTask.Sequence(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                ConcurrentTask.AllTasks(
                    new SpeakerAbsoluteOrientationTask(true, true),
                    new VisionShooterAimMasterTask()));
    }

    private OffboardVisionManager vision;
    private ArmMechanism arm;
    private EndEffectorMechanism endEffector;
    private SDSDriveTrainMechanism driveTrain;
    private DriverFeedbackManager driverFeedbackManager;


    private final LinearInterpolator angleLinterp;
    private final LinearInterpolator velocityLinterp;
    private Double distanceToSpeaker;
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
    
    public VisionShooterAimMasterTask()
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
        this.driverFeedbackManager = this.getInjector().getInstance(DriverFeedbackManager.class);
    }

    @Override 
    public void update()
    {
        if (this.currentState == State.FindSpeakerAprilTag) 
        {
            distanceToSpeaker = Math.sqrt(
                (Math.pow((vision.getAbsolutePositionX() - TuningConstants.CENTER_TO_SPEAKER_X_DISTANCE), 2)) +
                (Math.pow((vision.getAbsolutePositionY() - TuningConstants.CENTER_TO_SPEAKER_Y_DISTANCE), 2)));
            
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
            
            if ((Helpers.RoughEquals(driveTrain.getForwardFieldVelocity(), TuningConstants.SDSDRIVETRAIN_STATIONARY_VELOCITY, TuningConstants.ACCEPTABLE_NOT_MOVING_RANGE)) && 
            (Helpers.RoughEquals(driveTrain.getLeftFieldVelocity(), TuningConstants.SDSDRIVETRAIN_STATIONARY_VELOCITY, TuningConstants.ACCEPTABLE_NOT_MOVING_RANGE))) 
            {
                this.currentState = State.RumbleDriver;
            }
        }

        if(this.currentState == State.RumbleDriver)
        {
            driverFeedbackManager.setRumble(true);
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
