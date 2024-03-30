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
                ConcurrentTask.AnyTasks(
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                    new ShooterSpinTask(TuningConstants.SHOOT_VISION_SAMPLE_VELOCITIES[0], 20.0)),
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

    private double visionAbsX;
    private double visionAbsY;
    private double dtStartingX;
    private double dtStartingY;

    private boolean hasEverSeenTarget;
    private int noTargetCount;

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

        this.hasEverSeenTarget = false;
        this.distanceToSpeaker = null;

        this.visionAbsX = 0.0;
        this.visionAbsY = 0.0;
        this.dtStartingX = 0.0;
        this.dtStartingY = 0.0;

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
        Double absoluteRobotX = this.vision.getAbsolutePositionX();
        Double absoluteRobotY = this.vision.getAbsolutePositionY();

        if (absoluteRobotX == null || absoluteRobotY == null)
        {
            this.noTargetCount++;
            if (this.hasEverSeenTarget)
            {
                double xMovement = this.driveTrain.getPositionX() - this.dtStartingX;
                double yMovement = this.driveTrain.getPositionY() - this.dtStartingY;

                absoluteRobotX = this.visionAbsX + xMovement;
                absoluteRobotY = this.visionAbsY + yMovement;

                this.distanceToSpeaker = Math.sqrt(
                    Math.pow(absoluteRobotX - this.absoluteSpeakerX, 2) +
                    Math.pow(absoluteRobotY - this.absoluteSpeakerY, 2));
    
                this.wristAngle = this.angleLinterp.sample(this.distanceToSpeaker);
    
                double flywheelVelocity = this.velocityLinterp.sample(this.distanceToSpeaker);
                this.farFlywheelVelocity = flywheelVelocity;
                this.nearFlywheelVelocity = flywheelVelocity;
            }
        }
        else
        {
            this.visionAbsX = absoluteRobotX;
            this.visionAbsY = absoluteRobotY;

            this.dtStartingX = this.driveTrain.getPositionX();
            this.dtStartingY = this.driveTrain.getPositionY();

            this.distanceToSpeaker = Math.sqrt(
                Math.pow(absoluteRobotX - this.absoluteSpeakerX, 2) +
                Math.pow(absoluteRobotY - this.absoluteSpeakerY, 2));

            this.wristAngle = this.angleLinterp.sample(this.distanceToSpeaker);

            double flywheelVelocity = this.velocityLinterp.sample(this.distanceToSpeaker);
            this.farFlywheelVelocity = flywheelVelocity;
            this.nearFlywheelVelocity = flywheelVelocity;

            this.hasEverSeenTarget = true;
            this.noTargetCount = 0;
        }

        boolean shouldRumble = false;
        if (Helpers.RoughEquals(this.arm.getWristPosition(), this.wristAngle, TuningConstants.SHOOT_VISION_WRIST_ACCURACY_THRESHOLD) &&
            Helpers.RoughEquals(this.driveTrain.getForwardFieldVelocity(), TuningConstants.SDSDRIVETRAIN_STATIONARY_VELOCITY, TuningConstants.ACCEPTABLE_NOT_MOVING_RANGE) &&
            Helpers.RoughEquals(this.driveTrain.getLeftFieldVelocity(), TuningConstants.SDSDRIVETRAIN_STATIONARY_VELOCITY, TuningConstants.ACCEPTABLE_NOT_MOVING_RANGE))
        {
            shouldRumble = true;
        }

        this.setDigitalOperationState(DigitalOperation.ForceLightDriverRumble, shouldRumble);
        this.setDigitalOperationState(DigitalOperation.VisionFindAbsolutePosition, true);

        if (this.hasEverSeenTarget)
        {
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristAngle); 
            this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, this.nearFlywheelVelocity);
            this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, this.farFlywheelVelocity);
        }
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.ForceLightDriverRumble, false);
        this.setDigitalOperationState(DigitalOperation.VisionFindAbsolutePosition, false);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean shouldCancel()
    {
        if (!this.hasEverSeenTarget)
        {
            return this.noTargetCount > TuningConstants.SHOOT_VISION_ABSOLUTE_APRILTAG_NOT_FOUND_THRESHOLD;
        }

        return false;
    }

    @Override
    public boolean hasCompleted()
    {
        return false; // continuous
    }
}
