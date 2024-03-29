package frc.robot.driver.controltasks;

import java.util.Optional;

import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IRobotProvider;
import frc.robot.TuningConstants;
import frc.robot.mechanisms.OffboardVisionManager;
import frc.robot.mechanisms.PigeonManager;

public class SpeakerAbsoluteOrientationTask extends PIDTurnTaskBase
{
    private final boolean continuous;

    private double absoluteSpeakerX;
    private double absoluteSpeakerY;

    private Double absoluteRobotX;
    private Double absoluteRobotY;

    private boolean isRedAlliance;

    private PigeonManager pigeonManager;
    private OffboardVisionManager visionManager;

    public SpeakerAbsoluteOrientationTask(boolean continuous)
    {
        super(true, false);

        this.continuous = continuous;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        this.pigeonManager = this.getInjector().getInstance(PigeonManager.class);

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

        boolean canSeeAprilTag = absoluteRobotX != null && absoluteRobotY != null;
        if (!canSeeAprilTag &&
            (this.absoluteRobotX == null || this.absoluteRobotY == null))
        {
            // couldn't find an AprilTag
            return null;
        }

        if (canSeeAprilTag)
        {
            this.absoluteRobotX = absoluteRobotX;
            this.absoluteRobotY = absoluteRobotY;
        }

        double absoluteDeltaXSpeakerToRobot = this.absoluteRobotX - this.absoluteSpeakerX;
        double absoluteDeltaYSpeakerToRobot = this.absoluteRobotY - this.absoluteSpeakerY;
        
        double orientationTheta = this.pigeonManager.getYaw() - Helpers.atan2d(
            absoluteDeltaYSpeakerToRobot,
            absoluteDeltaXSpeakerToRobot);

        return this.isRedAlliance ? orientationTheta : -1.0 * orientationTheta;
    }
}
