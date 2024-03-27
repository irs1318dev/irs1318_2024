package frc.robot.driver.controltasks;
import java.util.Optional;

import frc.lib.driver.IControlTask;
import frc.lib.helpers.Helpers;
import frc.lib.helpers.LinearInterpolator;
import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IRobotProvider;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ArmMechanism;
import frc.robot.mechanisms.OffboardVisionManager;
import frc.robot.mechanisms.SDSDriveTrainMechanism;

public class VisionShooterAimMasterTask extends ControlTaskBase
{
    public static IControlTask createShootMacroTask()
    {
        return 
            SequentialTask.Sequence(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                ConcurrentTask.AllTasks(
                    new SpeakerAbsoluteOrientationTask(true),
                    new VisionShooterAimMasterTask()));
    }

    private double absoluteSpeakerX;
    private double absoluteSpeakerY;

    private OffboardVisionManager vision;
    private ArmMechanism arm;
    private SDSDriveTrainMechanism driveTrain;

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
        this.driveTrain = this.getInjector().getInstance(SDSDriveTrainMechanism.class);

        IRobotProvider provider = this.getInjector().getInstance(IRobotProvider.class);
        Optional<Alliance> alliance = provider.getDriverStation().getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red)
        {
            this.absoluteSpeakerX = TuningConstants.APRILTAG_RED_SPEAKER_X_POSITION;
            this.absoluteSpeakerY = TuningConstants.APRILTAG_RED_SPEAKER_Y_POSITION;
        }
        else
        {
            this.absoluteSpeakerX = TuningConstants.APRILTAG_BLUE_SPEAKER_X_POSITION;
            this.absoluteSpeakerY = TuningConstants.APRILTAG_BLUE_SPEAKER_Y_POSITION;
        }
    }

    @Override 
    public void update()
    {
        if (this.currentState == State.FindSpeakerAprilTag) 
        {
            Double absoluteRobotX = this.vision.getAbsolutePositionX();
            Double absoluteRobotY = this.vision.getAbsolutePositionY();

            if (absoluteRobotX == null || absoluteRobotY == null)
            {
                this.noTargetCount++;
                if (this.noTargetCount > TuningConstants.SHOOT_VISION_APRILTAG_NOT_FOUND_THRESHOLD)
                {
                    this.shouldCancel = true;
                }
            }
            else
            {
                distanceToSpeaker = Math.sqrt(
                    Math.pow(absoluteRobotX - this.absoluteSpeakerX, 2) +
                    Math.pow(absoluteRobotY - this.absoluteSpeakerY, 2));

                this.wristAngle = this.angleLinterp.sample(distanceToSpeaker);

                double flywheelVelocity = this.velocityLinterp.sample(distanceToSpeaker);
                this.farFlywheelVelocity = flywheelVelocity;
                this.nearFlywheelVelocity = flywheelVelocity;

                this.noTargetCount = 0;
                this.currentState = State.SetWristAndVelocity;
            }
        }

        boolean shouldRumble = false;
        if (this.currentState == State.SetWristAndVelocity)
        {
            if (Helpers.RoughEquals(this.arm.getWristPosition(), this.wristAngle, TuningConstants.SHOOT_VISION_WRIST_ACCURACY_THRESHOLD))
            {
                this.currentState = State.FindSpeakerAprilTag;
            }

            if (Helpers.RoughEquals(driveTrain.getForwardFieldVelocity(), TuningConstants.SDSDRIVETRAIN_STATIONARY_VELOCITY, TuningConstants.ACCEPTABLE_NOT_MOVING_RANGE) &&
                Helpers.RoughEquals(driveTrain.getLeftFieldVelocity(), TuningConstants.SDSDRIVETRAIN_STATIONARY_VELOCITY, TuningConstants.ACCEPTABLE_NOT_MOVING_RANGE))
            {
                shouldRumble = true;
            }
        }

        this.setDigitalOperationState(DigitalOperation.ForceLightDriverRumble, shouldRumble);
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
                break;

            default:
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
        return false; // continuous
    }
}
