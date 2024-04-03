package frc.robot.driver.controltasks;

import java.util.Optional;

import frc.lib.driver.IControlTask;
import frc.lib.filters.FadingMemoryFilter;
import frc.lib.filters.FloatingAverageCalculator;
import frc.lib.filters.ISimpleFilter;
import frc.lib.helpers.Helpers;
import frc.lib.helpers.LinearInterpolator;
import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ArmMechanism;
import frc.robot.mechanisms.EndEffectorMechanism;
import frc.robot.mechanisms.OffboardVisionManager;
import frc.robot.mechanisms.SDSDriveTrainMechanism;

public class VisionShooterTurnAndAimRelativeTask extends PIDTurnTaskBase
{
    public static IControlTask createShootMacroTask()
    {
        return 
            SequentialTask.Sequence(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                ConcurrentTask.AnyTasks(
                    new VisionSingleTurningTask(VisionSingleTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear), // centers to the apriltag we find, may be the off-centered one initially
                    new ShooterSpinTask(TuningConstants.SHOOT_VISION_SAMPLE_VELOCITIES[0], 10.0)),
                new VisionShooterTurnAndAimRelativeTask());
    }

    private final LinearInterpolator angleLinterp;
    private final LinearInterpolator velocityLinterp;

    private ISimpleFilter visionRelXFilter;
    private ISimpleFilter visionRelYFilter;

    private OffboardVisionManager visionManager;
    private ArmMechanism armMechanism;
    private SDSDriveTrainMechanism driveTrainMechanism;
    private EndEffectorMechanism endEffectorMechanism;

    private boolean isRedAlliance;

    private Double turnAngle;
    private Double wristAngle;
    private Double farFlywheelVelocity;
    private Double nearFlywheelVelocity;

    private int noTargetCount;
    private boolean hasEverSeenTarget;

    public VisionShooterTurnAndAimRelativeTask()
    {
        super(true, false, TuningConstants.SHOOT_VISION_ABSOLUTE_APRILTAG_NOT_FOUND_THRESHOLD);

        this.angleLinterp = new LinearInterpolator(TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES, TuningConstants.SHOOT_VISION_SAMPLE_ANGLES);
        this.velocityLinterp = new LinearInterpolator(TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES, TuningConstants.SHOOT_VISION_SAMPLE_VELOCITIES);
    }

    @Override
    public void begin()
    {
        // ITimer timer = this.getInjector().getInstance(ITimer.class);
        this.visionRelXFilter = new FadingMemoryFilter(0.0, 1.0); // new FloatingAverageCalculator(timer, 0.25, TuningConstants.LOOPS_PER_SECOND);
        this.visionRelYFilter = new FadingMemoryFilter(0.0, 1.0); // new FloatingAverageCalculator(timer, 0.25, TuningConstants.LOOPS_PER_SECOND);

        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        this.armMechanism = this.getInjector().getInstance(ArmMechanism.class);
        this.endEffectorMechanism = this.getInjector().getInstance(EndEffectorMechanism.class);
        this.driveTrainMechanism = this.getInjector().getInstance(SDSDriveTrainMechanism.class);

        this.noTargetCount = 0;
        this.hasEverSeenTarget = false;

        this.turnAngle = null;
        this.wristAngle = null;
        this.farFlywheelVelocity = null;
        this.nearFlywheelVelocity = null;

        this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, true);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);

        IRobotProvider provider = this.getInjector().getInstance(IRobotProvider.class);
        Optional<Alliance> alliance = provider.getDriverStation().getAlliance();

        this.isRedAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;

        super.begin();
    }

    @Override
    public void update()
    {
        // Note: we want to point toward the AprilTag, not match its yaw (make ourselves parallel to it), so we can use the fact that tan(angle) = opposite / adjacent
        Double xOffset = this.visionManager.getAprilTagXOffset();
        Double yOffset = this.visionManager.getAprilTagYOffset();
        Integer tagId = this.visionManager.getAprilTagId();

        if (xOffset == null || yOffset == null || tagId == null)
        {
            // for continuous turning - don't keep turning unless we still see the AprilTag
            this.turnAngle = null;

            this.noTargetCount++;
        }
        else
        {
            xOffset = this.visionRelXFilter.update(xOffset); // filter the data to avoid excessive noise
            yOffset = this.visionRelYFilter.update(yOffset); // filter the data to avoid excessive noise

            if (TuningConstants.SHOOT_VISION_RELATIVE_FIND_OFFCENTER_TAG)
            {
                if (this.isRedAlliance && tagId == TuningConstants.APRILTAG_RED_SPEAKER_OFFCENTER_ID)
                {
                    // TODO: aim 24" to the left of this april tag!
                }
                else if (!this.isRedAlliance && tagId == TuningConstants.APRILTAG_BLUE_SPEAKER_OFFCENTER_ID)
                {
                    // TODO: aim 24" to the right of this april tag!
                }
            }

            this.turnAngle = -Helpers.atan2d(yOffset, xOffset);

            double distance = Math.abs(xOffset);

            this.wristAngle = this.angleLinterp.sample(distance);

            double flywheelVelocity = this.velocityLinterp.sample(distance);
            this.farFlywheelVelocity = flywheelVelocity;
            this.nearFlywheelVelocity = flywheelVelocity;

            this.hasEverSeenTarget = true;
            this.noTargetCount = 0;
        }

        boolean shouldRumble = false;
        if (this.hasEverSeenTarget &&
            Helpers.RoughEquals(this.armMechanism.getWristPosition(), this.wristAngle, TuningConstants.SHOOT_VISION_WRIST_ACCURACY_THRESHOLD) &&
            Helpers.RoughEquals(this.driveTrainMechanism.getForwardFieldVelocity(), TuningConstants.SDSDRIVETRAIN_STATIONARY_VELOCITY, TuningConstants.ACCEPTABLE_NOT_MOVING_RANGE) &&
            Helpers.RoughEquals(this.driveTrainMechanism.getLeftFieldVelocity(), TuningConstants.SDSDRIVETRAIN_STATIONARY_VELOCITY, TuningConstants.ACCEPTABLE_NOT_MOVING_RANGE) &&
            this.endEffectorMechanism.isFlywheelSpunUp())
        {
            shouldRumble = true;
        }

        this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, true);
        if (this.hasEverSeenTarget)
        {
            this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, this.wristAngle);
            this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, this.nearFlywheelVelocity);
            this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, this.farFlywheelVelocity);
        }

        this.setDigitalOperationState(DigitalOperation.ForceLightDriverRumble, shouldRumble);

        super.update();
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, false);
        this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, TuningConstants.MAGIC_NULL_VALUE);
        this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean shouldCancel()
    {
        if (this.hasEverSeenTarget)
        {
            return this.noTargetCount > TuningConstants.SHOOT_VISION_ABSOLUTE_APRILTAG_NOT_FOUND_THRESHOLD;
        }

        return this.noTargetCount > TuningConstants.SHOOT_VISION_APRILTAG_NOT_FOUND_THRESHOLD;
    }

    @Override
    public boolean hasCompleted()
    {
        return false; // continuous
    }

    @Override
    protected Double getHorizontalAngle()
    {
        return this.turnAngle;
    }
}