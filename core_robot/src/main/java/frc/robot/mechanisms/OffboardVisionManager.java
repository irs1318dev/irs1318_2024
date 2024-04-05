package frc.robot.mechanisms;

import frc.robot.*;
import frc.lib.driver.IDriver;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.*;
import frc.robot.driver.*;

import java.util.List;
import java.util.Optional;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Offboard Vision manager.
 */
@Singleton
public class OffboardVisionManager implements IMechanism
{
    public static final DigitalOperation[] PossibleVisionOperations =
    {
        DigitalOperation.VisionFindAnyAprilTagFront,
        DigitalOperation.VisionFindAnyAprilTagRear,
        DigitalOperation.VisionFindSpeakerAprilTagFront,
        DigitalOperation.VisionFindSpeakerAprilTagRear,
        DigitalOperation.VisionFindAmpAprilTagFront,
        DigitalOperation.VisionFindAmpAprilTagRear,
        DigitalOperation.VisionFindStageAprilTagsFront,
        DigitalOperation.VisionFindStageAprilTagsRear,
        DigitalOperation.VisionFindAbsolutePosition,
    };

    public static final List<DigitalOperation> PossibleFrontVisionOperations =
        List.of(
            DigitalOperation.VisionFindAnyAprilTagFront,
            DigitalOperation.VisionFindSpeakerAprilTagFront,
            DigitalOperation.VisionFindAmpAprilTagFront,
            DigitalOperation.VisionFindStageAprilTagsFront);

    private final IDriver driver;
    private final ILogger logger;

    private final INetworkTableProvider networkTable;
    private final IDriverStation ds;

    private IDoubleSubscriber atrXOffsetSubscriber;
    private IDoubleSubscriber atrYOffsetSubscriber;
    private IDoubleSubscriber atrZOffsetSubscriber;
    private IDoubleSubscriber atrYawSubscriber;
    private IDoubleSubscriber atrPitchSubscriber;
    private IDoubleSubscriber atrRollSubscriber;
    private IDoubleSubscriber atrIdSubscriber;

    private IDoubleSubscriber atfXOffsetSubscriber;
    private IDoubleSubscriber atfYOffsetSubscriber;
    private IDoubleSubscriber atfZOffsetSubscriber;
    private IDoubleSubscriber atfYawSubscriber;
    private IDoubleSubscriber atfPitchSubscriber;
    private IDoubleSubscriber atfRollSubscriber;
    private IDoubleSubscriber atfIdSubscriber;

    private IDoubleSubscriber absXOffsetSubscriber;
    private IDoubleSubscriber absYOffsetSubscriber;
    private IDoubleSubscriber absZOffsetSubscriber;
    private IDoubleSubscriber absRollAngleSubscriber;
    private IDoubleSubscriber absPitchAngleSubscriber;
    private IDoubleSubscriber absYawAngleSubscriber;
    private IDoubleSubscriber absTagIdSubscriber;
    private IDoubleSubscriber absDecisionMarginSubscriber;
    private IDoubleSubscriber absErrorSubscriber;

    private IDoubleSubscriber heartbeatSubscriber;

    private Double atXOffset;
    private Double atYOffset;
    private Double atZOffset;
    private Double atYaw;
    private Double atPitch;
    private Double atRoll;
    private Integer atId;

    private Double absXOffset;
    private Double absYOffset;
    private Double absZOffset;
    private Double absRollAngle;
    private Double absPitchAngle;
    private Double absYawAngle;
    private Integer absTagId;
    private Double absDecisionMargin;
    private Double absError;

    private int prevMode;
    private List<Integer> prevTargets;

    private int missedHeartbeats;
    private long prevHeartbeat;

    /**
     * Initializes a new OffboardVisionManager
     * @param driver for obtaining operations
     * @param logger for logging to smart dashboard
     * @param provider for obtaining electronics objects
     */
    @Inject
    public OffboardVisionManager(IDriver driver, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;

        this.networkTable = provider.getNetworkTableProvider();

        this.atrXOffsetSubscriber = this.networkTable.getDoubleSubscriber("atr.xOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atrYOffsetSubscriber = this.networkTable.getDoubleSubscriber("atr.yOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atrZOffsetSubscriber = this.networkTable.getDoubleSubscriber("atr.zOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atrYawSubscriber = this.networkTable.getDoubleSubscriber("atr.yawAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atrPitchSubscriber = this.networkTable.getDoubleSubscriber("atr.pitchAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atrRollSubscriber = this.networkTable.getDoubleSubscriber("atr.rollAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atrIdSubscriber = this.networkTable.getDoubleSubscriber("atr.tagId", (int)TuningConstants.MAGIC_NULL_VALUE);

        this.atfXOffsetSubscriber = this.networkTable.getDoubleSubscriber("atf.xOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atfYOffsetSubscriber = this.networkTable.getDoubleSubscriber("atf.yOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atfZOffsetSubscriber = this.networkTable.getDoubleSubscriber("atf.zOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atfYawSubscriber = this.networkTable.getDoubleSubscriber("atf.yawAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atfPitchSubscriber = this.networkTable.getDoubleSubscriber("atf.pitchAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atfRollSubscriber = this.networkTable.getDoubleSubscriber("atf.rollAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atfIdSubscriber = this.networkTable.getDoubleSubscriber("atf.tagId", (int)TuningConstants.MAGIC_NULL_VALUE);

        this.absXOffsetSubscriber = this.networkTable.getDoubleSubscriber("abs.xOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.absYOffsetSubscriber = this.networkTable.getDoubleSubscriber("abs.yOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.absZOffsetSubscriber = this.networkTable.getDoubleSubscriber("abs.zOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.absRollAngleSubscriber = this.networkTable.getDoubleSubscriber("abs.rollAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.absPitchAngleSubscriber = this.networkTable.getDoubleSubscriber("abs.pitchAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.absYawAngleSubscriber = this.networkTable.getDoubleSubscriber("abs.yawAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.absTagIdSubscriber = this.networkTable.getDoubleSubscriber("abs.tagId", TuningConstants.MAGIC_NULL_VALUE);
        this.absDecisionMarginSubscriber = this.networkTable.getDoubleSubscriber("abs.decisionMargin", TuningConstants.MAGIC_NULL_VALUE);
        this.absErrorSubscriber = this.networkTable.getDoubleSubscriber("abs.error", TuningConstants.MAGIC_NULL_VALUE);

        this.heartbeatSubscriber = this.networkTable.getDoubleSubscriber("v.heartbeat", 0);

        this.ds = provider.getDriverStation();

        this.atXOffset = null;
        this.atYOffset = null;
        this.atZOffset = null;
        this.atYaw = null;
        this.atPitch = null;
        this.atRoll = null;
        this.atId = null;

        this.prevMode = 0;
        this.prevTargets = null;

        this.missedHeartbeats = 0;
        this.prevHeartbeat = 0L;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        double atrXOffset = this.atrXOffsetSubscriber.get();
        double atrYOffset = this.atrYOffsetSubscriber.get();
        double atrZOffset = this.atrZOffsetSubscriber.get();
        double atrYaw = this.atrYawSubscriber.get();
        double atrPitch = this.atrPitchSubscriber.get();
        double atrRoll = this.atrRollSubscriber.get();
        int atrId = (int)this.atrIdSubscriber.get();

        double atfXOffset = this.atfXOffsetSubscriber.get();
        double atfYOffset = this.atfYOffsetSubscriber.get();
        double atfZOffset = this.atfZOffsetSubscriber.get();
        double atfYaw = this.atfYawSubscriber.get();
        double atfPitch = this.atfPitchSubscriber.get();
        double atfRoll = this.atfRollSubscriber.get();
        int atfId = (int)this.atfIdSubscriber.get();

        double absXOffset = this.absXOffsetSubscriber.get();
        double absYOffset = this.absYOffsetSubscriber.get();
        double absZOffset = this.absZOffsetSubscriber.get();
        double absRollAngle = this.absRollAngleSubscriber.get();
        double absPitchAngle = this.absPitchAngleSubscriber.get();
        double absYawAngle = this.absYawAngleSubscriber.get();
        int absTagId = (int)this.absTagIdSubscriber.get();
        double absDecisionMargin = this.absDecisionMarginSubscriber.get();
        double absError = this.absErrorSubscriber.get();

        double newHeartbeat = this.heartbeatSubscriber.get();
        if (this.prevHeartbeat != newHeartbeat)
        {
            this.missedHeartbeats = 0;
        }
        else
        {
            this.missedHeartbeats++;
        }

        this.logger.logNumber(LoggingKey.OffboardVisionMissedHeartbeats, this.missedHeartbeats);

        boolean missedHeartbeatExceedsThreshold = this.missedHeartbeats > TuningConstants.VISION_MISSED_HEARTBEAT_THRESHOLD;
        this.logger.logBoolean(LoggingKey.OffboardVisionExcessiveMissedHeartbeats, missedHeartbeatExceedsThreshold);

        // reset if we couldn't find the april tag
        this.atXOffset = null;
        this.atYOffset = null;
        this.atZOffset = null;
        this.atYaw = null;
        this.atPitch = null;
        this.atRoll = null;
        this.atId = null;

        this.absXOffset = null;
        this.absYOffset = null;
        this.absZOffset = null;
        this.absRollAngle = null;
        this.absPitchAngle = null;
        this.absYawAngle = null;
        this.absTagId = null;
        this.absDecisionMargin = null;
        this.absError = null;

        if (!missedHeartbeatExceedsThreshold)
        {
            switch (this.prevMode)
            {
                case 1:
                    if (atrXOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        atrYOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        atrZOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        (this.prevTargets == null || this.prevTargets.contains(atrId)))
                    {
                        this.atXOffset = atrXOffset;
                        this.atYOffset = atrYOffset;
                        this.atZOffset = atrZOffset;
                        this.atYaw = atrYaw;
                        this.atPitch = atrPitch;
                        this.atRoll = atrRoll;
                        this.atId = atrId;
                    }

                    break;

                case 2:
                    if (atfXOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        atfYOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        atfZOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        (this.prevTargets == null || this.prevTargets.contains(atfId)))
                    {
                        this.atXOffset = atfXOffset;
                        this.atYOffset = atfYOffset;
                        this.atZOffset = atfZOffset;
                        this.atYaw = atfYaw;
                        this.atPitch = atfPitch;
                        this.atRoll = atfRoll;
                        this.atId = atfId;
                    }

                    break;

                case 3:
                    if (absXOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        absYOffset != TuningConstants.MAGIC_NULL_VALUE &&
                        absZOffset != TuningConstants.MAGIC_NULL_VALUE)
                    {
                        this.absXOffset = absXOffset;
                        this.absYOffset = absYOffset;
                        this.absZOffset = absZOffset;
                        this.absRollAngle = absRollAngle;
                        this.absPitchAngle = absPitchAngle;
                        this.absYawAngle = absYawAngle;
                        this.absTagId = absTagId;
                        this.absDecisionMargin = absDecisionMargin;
                        this.absError = absError;
                    }

                case 0:
                default:
                    break;
            }
        }

        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagXOffset, this.atXOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagYOffset, this.atYOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagZOffset, this.atZOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagYaw, this.atYaw);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagPitch, this.atPitch);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagRoll, this.atRoll);
        this.logger.logInteger(LoggingKey.OffboardVisionAprilTagId, this.atId);
    }

    @Override
    public void update(RobotMode mode)
    {
        boolean enableVision = !this.driver.getDigital(DigitalOperation.VisionForceDisable);
        boolean enableVideoStream = mode == RobotMode.Test || this.driver.getDigital(DigitalOperation.VisionEnableStream);
        boolean enableAnyRear = this.driver.getDigital(DigitalOperation.VisionFindAnyAprilTagRear);
        boolean enableAnyFront = this.driver.getDigital(DigitalOperation.VisionFindAnyAprilTagFront);
        boolean enableSpeakerRear = this.driver.getDigital(DigitalOperation.VisionFindSpeakerAprilTagRear);
        boolean enableSpeakerFront = this.driver.getDigital(DigitalOperation.VisionFindSpeakerAprilTagFront);
        boolean enableAmpRear = this.driver.getDigital(DigitalOperation.VisionFindAmpAprilTagRear);
        boolean enableAmpFront = this.driver.getDigital(DigitalOperation.VisionFindAmpAprilTagFront);
        boolean enableStageRear = this.driver.getDigital(DigitalOperation.VisionFindStageAprilTagsRear);
        boolean enableStageFront = this.driver.getDigital(DigitalOperation.VisionFindStageAprilTagsFront);
        boolean enableSourceFront = this.driver.getDigital(DigitalOperation.VisionFindSourceAprilTagsFront);
        boolean enableAbsolutePosition = this.driver.getDigital(DigitalOperation.VisionFindAbsolutePosition);

        Optional<Alliance> alliance = this.ds.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

        List<Integer> desiredTargets = null;
        String desiredTargetsString = "";
        int visionProcessingMode = 0;
        if (enableVision)
        {
            if (enableAnyRear || enableSpeakerRear || enableStageRear || enableAmpRear)
            {
                visionProcessingMode = 1;
            }
            else if (enableAnyFront || enableSpeakerFront || enableStageFront || enableAmpFront || enableSourceFront)
            {
                visionProcessingMode = 2;
            }
            else if (enableAbsolutePosition)
            {
                visionProcessingMode = 3;
            }

            if (enableSpeakerFront || enableSpeakerRear)
            {
                if (isRed)
                {
                    desiredTargets = TuningConstants.VISION_SPEAKER_RED_APRILTAGS;
                    desiredTargetsString = TuningConstants.VISION_SPEAKER_RED_STRING;
                }
                else
                {
                    desiredTargets = TuningConstants.VISION_SPEAKER_BLUE_APRILTAGS;
                    desiredTargetsString = TuningConstants.VISION_SPEAKER_BLUE_STRING;
                }
            }

            if (enableStageFront || enableStageRear)
            {
                if (isRed)
                {
                    desiredTargets = TuningConstants.VISION_STAGE_RED_APRILTAGS;
                    desiredTargetsString = TuningConstants.VISION_STAGE_RED_STRING;
                }
                else
                {
                    desiredTargets = TuningConstants.VISION_STAGE_BLUE_APRILTAGS;
                    desiredTargetsString = TuningConstants.VISION_STAGE_BLUE_STRING;
                }
            }

            if (enableAmpFront || enableAmpRear)
            {
                if (isRed)
                {
                    desiredTargets = TuningConstants.VISION_AMP_RED_APRILTAGS;
                    desiredTargetsString = TuningConstants.VISION_AMP_RED_STRING;
                }
                else
                {
                    desiredTargets = TuningConstants.VISION_AMP_BLUE_APRILTAGS;
                    desiredTargetsString = TuningConstants.VISION_AMP_BLUE_STRING;
                }
            }
        }

        this.prevMode = visionProcessingMode;
        this.prevTargets = desiredTargets;
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, enableVideoStream);
        this.logger.logInteger(LoggingKey.OffboardVisionProcessingMode, visionProcessingMode);
        this.logger.logString(LoggingKey.OffboardVisionDesiredTarget, desiredTargetsString);
    }

    @Override
    public void stop()
    {
        this.prevMode = 0;
        this.prevTargets = null;
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, false);
        this.logger.logInteger(LoggingKey.OffboardVisionProcessingMode, 0);
        this.logger.logString(LoggingKey.OffboardVisionDesiredTarget, "");
    }

    public Double getAprilTagXOffset()
    {
        return this.atXOffset;
    }

    public Double getAprilTagYOffset()
    {
        return this.atYOffset;
    }

    public Double getAprilTagZOffset()
    {
        return this.atZOffset;
    }

    public Double getAprilTagYaw()
    {
        return this.atYaw;
    }

    public Double getAprilTagPitch()
    {
        return this.atPitch;
    }

    public Double getAprilTagRoll()
    {
        return this.atRoll;
    }

    public Integer getAprilTagId()
    {
        return this.atId;
    }

    public Double getAbsolutePositionX()
    {
        return this.absXOffset;
    }

    public Double getAbsolutePositionY()
    {
        return this.absYOffset;
    }

    public Double getAbsolutePositionZ()
    {
        return this.absZOffset;
    }

    public Double getAbsolutePositionYaw()
    {
        return this.absYawAngle;
    }
}
