package frc.robot.driver.controltasks;

import java.util.Optional;

import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IRobotProvider;
import frc.robot.TuningConstants;
import frc.robot.mechanisms.OffboardVisionManager;
import frc.robot.mechanisms.PigeonManager;
import frc.robot.mechanisms.SDSDriveTrainMechanism;

public class SpeakerAbsoluteOrientationTask extends PIDTurnTaskBase
{
    private final boolean continuous;

    private double absoluteSpeakerX;
    private double absoluteSpeakerY;

    private double visionAbsX;
    private double visionAbsY;
    private double dtStartingX;
    private double dtStartingY;

    private boolean isRedAlliance;

    private PigeonManager pigeonManager;
    private OffboardVisionManager visionManager;
    private SDSDriveTrainMechanism driveTrain;

    private boolean hasEverSeenTarget;

    public SpeakerAbsoluteOrientationTask(boolean continuous)
    {
        super(true, false, TuningConstants.SHOOT_VISION_ABSOLUTE_APRILTAG_NOT_FOUND_THRESHOLD);

        this.continuous = continuous;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        this.pigeonManager = this.getInjector().getInstance(PigeonManager.class);
        this.driveTrain = this.getInjector().getInstance(SDSDriveTrainMechanism.class);

        this.visionAbsX = 0.0;
        this.visionAbsY = 0.0;
        this.dtStartingX = 0.0;
        this.dtStartingY = 0.0;

        this.hasEverSeenTarget = false;

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
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.continuous)
        {
            return false;
        }

        return super.hasCompleted();
    }

    @Override
    protected Double getHorizontalAngle()
    {
        Double absoluteRobotX = this.visionManager.getAbsolutePositionX();
        Double absoluteRobotY = this.visionManager.getAbsolutePositionY();

        double orientationTheta;
        if (absoluteRobotX != null && absoluteRobotY != null)
        {
            this.visionAbsX = absoluteRobotX;
            this.visionAbsY = absoluteRobotY;

            this.dtStartingX = this.driveTrain.getPositionX();
            this.dtStartingY = this.driveTrain.getPositionY();

            double absoluteDeltaXSpeakerToRobot = absoluteRobotX - this.absoluteSpeakerX;
            double absoluteDeltaYSpeakerToRobot = absoluteRobotY - this.absoluteSpeakerY;

            orientationTheta = 
                this.pigeonManager.getYaw() -
                    Helpers.atan2d(absoluteDeltaYSpeakerToRobot, absoluteDeltaXSpeakerToRobot);

            this.hasEverSeenTarget = true;
        }
        else if (hasEverSeenTarget)
        {
            double xMovement = this.driveTrain.getPositionX() - this.dtStartingX;
            double yMovement = this.driveTrain.getPositionY() - this.dtStartingY;

            absoluteRobotX = this.visionAbsX + xMovement;
            absoluteRobotY = this.visionAbsY + yMovement;

            double absoluteDeltaXSpeakerToRobot = absoluteRobotX - this.absoluteSpeakerX;
            double absoluteDeltaYSpeakerToRobot = absoluteRobotY - this.absoluteSpeakerY;

            orientationTheta = 
                this.pigeonManager.getYaw() -
                    Helpers.atan2d(absoluteDeltaYSpeakerToRobot, absoluteDeltaXSpeakerToRobot);
        }
        else
        {
            return null;
        }

        return this.isRedAlliance ? orientationTheta : -1.0 * orientationTheta;
    }
}
