package frc.robot.driver.controltasks;
import java.util.Optional;

import frc.lib.driver.IControlTask;
import frc.lib.filters.ComplementaryFilter;
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
import frc.robot.mechanisms.OffboardVisionManager;
import frc.robot.mechanisms.PigeonManager;
import frc.robot.mechanisms.SDSDriveTrainMechanism;

public class VisionShooterTurnAndAimAbsoluteTask extends PIDTurnTaskBase
{
    private static final int NUM_UPDATES_TO_SKIP = 25;
    public static IControlTask createShootMacroTask()
    {
        return 
            SequentialTask.Sequence(
                ConcurrentTask.AnyTasks(
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                    new ShooterSpinTask(TuningConstants.SHOOT_VISION_SAMPLE_VELOCITIES[0], 20.0)),
                new VisionShooterTurnAndAimAbsoluteTask());
    }

    private final LinearInterpolator angleLinterp;
    private final LinearInterpolator velocityLinterp;

    private ISimpleFilter visionAbsXFilter;
    private ISimpleFilter visionAbsYFilter;

    private OffboardVisionManager vision;
    private ArmMechanism arm;
    private SDSDriveTrainMechanism driveTrain;
    private PigeonManager pigeonManager;

    private boolean isRedAlliance;

    private double absoluteSpeakerX;
    private double absoluteSpeakerY;

    private Double distanceToSpeaker;
    private Double orientationTheta;
    private double farFlywheelVelocity;
    private double nearFlywheelVelocity;
    private double wristAngle;

    private double visionAbsX;
    private double visionAbsY;
    private double dtStartingX;
    private double dtStartingY;

    private int skippedUpdates;
    private boolean hasEverSeenTarget;
    private int noTargetCount;

    public VisionShooterTurnAndAimAbsoluteTask()
    {
        super(true, false, TuningConstants.SHOOT_VISION_ABSOLUTE_APRILTAG_NOT_FOUND_THRESHOLD);

        this.angleLinterp = new LinearInterpolator(TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES, TuningConstants.SHOOT_VISION_SAMPLE_ANGLES);
        this.velocityLinterp = new LinearInterpolator(TuningConstants.SHOOT_VISION_SAMPLE_DISTANCES, TuningConstants.SHOOT_VISION_SAMPLE_VELOCITIES);
    }

    @Override
    public void begin()
    {
        super.begin();

        ITimer timer = this.getInjector().getInstance(ITimer.class);
        this.visionAbsXFilter = new FloatingAverageCalculator(timer, 0.5, TuningConstants.LOOPS_PER_SECOND); //new ComplementaryFilter(0.5, 0.5);
        this.visionAbsYFilter = new FloatingAverageCalculator(timer, 0.5, TuningConstants.LOOPS_PER_SECOND); //new ComplementaryFilter(0.5, 0.5);

        this.vision = this.getInjector().getInstance(OffboardVisionManager.class);
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        this.driveTrain = this.getInjector().getInstance(SDSDriveTrainMechanism.class);
        this.pigeonManager = this.getInjector().getInstance(PigeonManager.class);

        this.hasEverSeenTarget = false;
        this.skippedUpdates = 0;
        this.distanceToSpeaker = null;
        this.orientationTheta = null;

        this.visionAbsX = 0.0;
        this.visionAbsY = 0.0;
        this.dtStartingX = 0.0;
        this.dtStartingY = 0.0;

        IRobotProvider provider = this.getInjector().getInstance(IRobotProvider.class);
        Optional<Alliance> alliance = provider.getDriverStation().getAlliance();

        this.isRedAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;
        if (this.isRedAlliance)
        {
            this.absoluteSpeakerX = TuningConstants.APRILTAG_RED_SPEAKER_X_POSITION;
            this.absoluteSpeakerY = TuningConstants.APRILTAG_RED_SPEAKER_Y_POSITION;
        }
        else
        {
            this.absoluteSpeakerX = TuningConstants.APRILTAG_BLUE_SPEAKER_X_POSITION;
            this.absoluteSpeakerY = TuningConstants.APRILTAG_BLUE_SPEAKER_Y_POSITION;
        }

        this.visionAbsXFilter.reset();
        this.visionAbsYFilter.reset();
    }

    @Override
    public void update()
    {
        Double absoluteRobotX = this.vision.getAbsolutePositionX();
        Double absoluteRobotY = this.vision.getAbsolutePositionY();

        boolean recalculateOdometry = false; // whether to recalculate our desired angles, etc. based on odometry
        if (absoluteRobotX == null || absoluteRobotY == null)
        {
            this.noTargetCount++;
            if (this.hasEverSeenTarget)
            {
                recalculateOdometry = true;
            }
        }
        else
        {
            double dtX = this.driveTrain.getPositionX();
            double dtY = this.driveTrain.getPositionY();
            if (!this.hasEverSeenTarget ||
                (this.skippedUpdates % VisionShooterTurnAndAimAbsoluteTask.NUM_UPDATES_TO_SKIP) == 0)
            {
                this.visionAbsX = this.visionAbsXFilter.update(absoluteRobotX);
                this.visionAbsY = this.visionAbsYFilter.update(absoluteRobotY);
                if (!this.hasEverSeenTarget)
                {
                    this.visionAbsX = absoluteRobotX;
                    this.visionAbsY = absoluteRobotY;
                }

                this.dtStartingX = dtX;
                this.dtStartingY = dtY;

                double absoluteDeltaXSpeakerToRobot = absoluteRobotX - this.absoluteSpeakerX;
                double absoluteDeltaYSpeakerToRobot = absoluteRobotY - this.absoluteSpeakerY;

                double goalAngle = Helpers.atan2d(absoluteDeltaYSpeakerToRobot, absoluteDeltaXSpeakerToRobot);
                if (this.isRedAlliance)
                {
                    goalAngle = 180.0 + goalAngle;
                }
    
                this.orientationTheta = Helpers.updateAngleRange180(this.pigeonManager.getYaw() - goalAngle);
    
                this.distanceToSpeaker = Math.sqrt(
                    Math.pow(absoluteDeltaXSpeakerToRobot, 2) +
                    Math.pow(absoluteDeltaYSpeakerToRobot, 2));

                System.out.println(String.format("(from new): goal pos %f", goalAngle));
                System.out.println(String.format("(from new): distance %f", this.distanceToSpeaker));
    
                this.wristAngle = this.angleLinterp.sample(this.distanceToSpeaker);
    
                double flywheelVelocity = this.velocityLinterp.sample(this.distanceToSpeaker);
                this.farFlywheelVelocity = flywheelVelocity;
                this.nearFlywheelVelocity = flywheelVelocity;
    
                this.hasEverSeenTarget = true;
                this.skippedUpdates = 1;
            }
            else
            {
                this.visionAbsXFilter.update(absoluteRobotX);
                this.visionAbsYFilter.update(absoluteRobotY);

                this.skippedUpdates++;
                recalculateOdometry = true;
            }

            this.noTargetCount = 0;
        }

        if (recalculateOdometry)
        {
            double xMovement = this.driveTrain.getPositionX() - this.dtStartingX;
            double yMovement = this.driveTrain.getPositionY() - this.dtStartingY;

            absoluteRobotX = this.visionAbsX + xMovement;
            absoluteRobotY = this.visionAbsY + yMovement;

            double absoluteDeltaXSpeakerToRobot = absoluteRobotX - this.absoluteSpeakerX;
            double absoluteDeltaYSpeakerToRobot = absoluteRobotY - this.absoluteSpeakerY;

            double goalAngle = Helpers.atan2d(absoluteDeltaYSpeakerToRobot, absoluteDeltaXSpeakerToRobot);
            if (this.isRedAlliance)
            {
                goalAngle = 180.0 + goalAngle;
            }

            this.orientationTheta = Helpers.updateAngleRange180(this.pigeonManager.getYaw() - goalAngle);

            this.distanceToSpeaker = Math.sqrt(
                Math.pow(absoluteDeltaXSpeakerToRobot, 2) +
                Math.pow(absoluteDeltaYSpeakerToRobot, 2));

            System.out.println(String.format("(from previous): goal pos %f", goalAngle));
            System.out.println(String.format("(from previous): distance %f", this.distanceToSpeaker));

            this.wristAngle = this.angleLinterp.sample(this.distanceToSpeaker);

            double flywheelVelocity = this.velocityLinterp.sample(this.distanceToSpeaker);
            this.farFlywheelVelocity = flywheelVelocity;
            this.nearFlywheelVelocity = flywheelVelocity;
        }

        boolean shouldRumble = false;
        if (this.hasEverSeenTarget &&
            Helpers.RoughEquals(this.arm.getWristPosition(), this.wristAngle, TuningConstants.SHOOT_VISION_WRIST_ACCURACY_THRESHOLD) &&
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

        super.update();
    }

    @Override
    protected Double getHorizontalAngle()
    {
        return this.orientationTheta;
    }

    @Override
    public void end()
    {
        super.end();

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
