package frc.robot.driver.controltasks;

import frc.lib.driver.IControlTask;
import frc.lib.helpers.Helpers;
import frc.lib.helpers.LinearInterpolator;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ArmMechanism;
import frc.robot.mechanisms.EndEffectorMechanism;
import frc.robot.mechanisms.OffboardVisionManager;

public class VisionShooterAimLinterpTask extends ControlTaskBase
{
    public static IControlTask createShootMacroTask(boolean continuous)
    {
        if (continuous)
        {
            return ConcurrentTask.AnyTasks(
                new RumbleTask(),
                SequentialTask.Sequence(
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                    ConcurrentTask.AllTasks(
                        new VisionSingleTurningTask(VisionSingleTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear),
                        new VisionShooterAimLinterpTask(false)),
                    ConcurrentTask.AnyTasks(
                        new VisionContinuousTurningTask(VisionContinuousTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear, true),
                        new VisionShooterAimLinterpTask(true))));
        }

        return ConcurrentTask.AnyTasks(
            new RumbleTask(),
            new ShooterSpinTask(TuningConstants.SHOOT_VISION_SPEED, 15.0),
            SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new VisionSingleTurningTask(VisionSingleTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear),
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                new VisionShooterAimLinterpTask(false))));
    }

    public static IControlTask createFullShootMacroTask(boolean continuous)
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
        SetWristAndVelocity,
        Completed
    }

    private final LinearInterpolator angleLinterp;
    private final LinearInterpolator velocityLinterp;
    private final boolean continuous;

    private OffboardVisionManager vision;
    private ArmMechanism arm;
    private EndEffectorMechanism endEffector;

    private State currentState;
    private double wristAngle;
    private boolean shouldCancel;
    private int noTargetCount;
    private double farFlywheelVelocity;
    private double nearFlywheelVelocity;


    public VisionShooterAimLinterpTask(boolean continuous)
    {
        super();

        this.angleLinterp = new LinearInterpolator(TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES, TuningConstants.SHOOT_VISION_SAMPLE_ANGLES);
        this.velocityLinterp = new LinearInterpolator(TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES, TuningConstants.SHOOT_VISION_SAMPLE_VELOCITIES);
        this.continuous = continuous;
    }

    @Override
    public void begin()
    {
        this.vision = this.getInjector().getInstance(OffboardVisionManager.class);
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        this.endEffector = this.getInjector().getInstance(EndEffectorMechanism.class);
        this.currentState = State.FindSpeakerAprilTag;
        this.wristAngle = TuningConstants.MAGIC_NULL_VALUE;
        this.noTargetCount = 0;
        this.farFlywheelVelocity = TuningConstants.MAGIC_NULL_VALUE;
        this.nearFlywheelVelocity = TuningConstants.MAGIC_NULL_VALUE;

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
            // else if (!this.continuous &&
            //     (distance < TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES[0] ||
            //         distance > TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES[TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES.length - 1]))
            // {
            //     // if distance is out of range cancel, avoid inaccuracy in interpolation
            //     this.shouldCancel = true;
            // }
            else
            {
                this.wristAngle = this.angleLinterp.sample(distance);
                this.farFlywheelVelocity = this.velocityLinterp.sample(distance);
                this.nearFlywheelVelocity = this.velocityLinterp.sample(distance);
                this.currentState = State.SetWristAndVelocity;
            }
        }

        if (this.currentState == State.SetWristAndVelocity)
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
                if (this.continuous)
                {
                    this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, this.nearFlywheelVelocity);
                    this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, this.farFlywheelVelocity);
                }

                break;

            case SetWristAndVelocity:
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristAngle);
                if (this.continuous)
                {
                    this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, this.nearFlywheelVelocity);
                    this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, this.farFlywheelVelocity);
                }

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