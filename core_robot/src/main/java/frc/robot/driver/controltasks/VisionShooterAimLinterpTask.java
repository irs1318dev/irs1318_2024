package frc.robot.driver.controltasks;

import frc.lib.driver.IControlTask;
import frc.lib.helpers.Helpers;
import frc.lib.helpers.LinearInterpolator;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ArmMechanism;
import frc.robot.mechanisms.OffboardVisionManager;

public class VisionShooterAimLinterpTask extends ControlTaskBase
{
    public static IControlTask createShootMacroTask(boolean continuous)
    {
        if (continuous)
        {
            return ConcurrentTask.AnyTasks(
                new RumbleTask(),
                new ShooterSpinTask(TuningConstants.SHOOT_VISION_SPEED, 15.0),
                SequentialTask.Sequence(
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                    ConcurrentTask.AllTasks(
                        new VisionSingleTurningTask(VisionSingleTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear),
                        new VisionShooterAimLinterpTask(false)),
                    ConcurrentTask.AnyTasks(
                        new VisionContinuousTurningTask(VisionContinuousTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear, true),
                        new VisionShooterAimLinterpTask(true),
                        new FeedRingTask(true, 5.0))));
        }

        return ConcurrentTask.AnyTasks(
            new RumbleTask(),
            new ShooterSpinTask(TuningConstants.SHOOT_VISION_SPEED, 15.0),
            SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new VisionSingleTurningTask(VisionSingleTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear),
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                new VisionShooterAimLinterpTask(false),
                new FeedRingTask(true, 5.0))));
    }

    private enum State
    {
        FindSpeakerAprilTag,
        SetWristAngle,
        Completed
    }

    private final LinearInterpolator linterp;
    private final boolean continuous;

    private OffboardVisionManager vision;
    private ArmMechanism arm;

    private State currentState;
    private double wristAngle;
    private boolean shouldCancel;
    private int noTargetCount;

    public VisionShooterAimLinterpTask(boolean continuous)
    {
        super();

        this.linterp = new LinearInterpolator(TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES, TuningConstants.SHOOT_VISION_SAMPLE_ANGLES);
        this.continuous = continuous;
    }

    @Override
    public void begin()
    {
        this.vision = this.getInjector().getInstance(OffboardVisionManager.class);
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        this.currentState = State.FindSpeakerAprilTag;
        this.wristAngle = 0.0;
        this.noTargetCount = 0;

        this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, true);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
    }


    @Override
    public void update()
    {
        if (this.currentState == State.FindSpeakerAprilTag)
        {
            Double distance = this.vision.getAprilTagXOffset();
            if (distance == null)
            {
                this.noTargetCount++;
                if (this.noTargetCount > TuningConstants.SHOOT_VISION_APRILTAG_NOT_FOUND_THRESHOLD)
                {
                    this.shouldCancel = true;
                }
            }

            // if distance is out of range cancel, avoid inaccuracy in interpolation
            if (!this.continuous &&
                (distance < TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES[0] ||
                    distance > TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES[TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES.length - 1]))
            {
                this.shouldCancel = true;
            }
            else
            {
                this.wristAngle = this.linterp.sample(distance);
                this.currentState = State.SetWristAngle;
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristAngle);
            }
        }

        if (this.currentState == State.SetWristAngle)
        {
            this.noTargetCount = 0;
            if (Helpers.RoughEquals(this.arm.getWristPosition(), this.wristAngle, TuningConstants.SHOOT_VISION_WRIST_ACCURACY_THRESHOLD))
            {
                if (this.continuous)
                {
                    this.currentState = State.FindSpeakerAprilTag;
                }
                else
                {
                    this.currentState = State.Completed;
                }
            }
        }

        switch (this.currentState)
        {
            case FindSpeakerAprilTag:
                this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, true);
                break;

            case SetWristAngle:
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristAngle);
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