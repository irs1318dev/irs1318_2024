package frc.robot.mechanisms;

import frc.robot.*;
import frc.lib.driver.IDriver;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.*;
import frc.robot.driver.*;

import java.util.Optional;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Offboard Vision manager.
 */
@Singleton
public class OffboardVisionManager implements IMechanism
{
    private final IDriver driver;
    private final ILogger logger;

    private final INetworkTableProvider networkTable;
    private final IDriverStation ds;

    private IDoubleSubscriber atXOffsetSubscriber;
    private IDoubleSubscriber atYOffsetSubscriber;
    private IDoubleSubscriber atZOffsetSubscriber;
    private IDoubleSubscriber atYawSubscriber;
    private IDoubleSubscriber atPitchSubscriber;
    private IDoubleSubscriber atRollSubscriber;
    private IIntegerSubscriber atIdSubscriber;
    private IIntegerSubscriber heartbeatSubscriber;

    private Double atXOffset;
    private Double atYOffset;
    private Double atZOffset;
    private Double atYaw;
    private Double atPitch;
    private Double atRoll;
    private Integer atId;

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
        this.atXOffsetSubscriber = this.networkTable.getDoubleSubscriber("at.xOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atYOffsetSubscriber = this.networkTable.getDoubleSubscriber("at.yOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atZOffsetSubscriber = this.networkTable.getDoubleSubscriber("at.zOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atYawSubscriber = this.networkTable.getDoubleSubscriber("at.yawAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atPitchSubscriber = this.networkTable.getDoubleSubscriber("at.pitchAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atRollSubscriber = this.networkTable.getDoubleSubscriber("at.rollAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atIdSubscriber = this.networkTable.getIntegerSubscriber("at.tagId", (int) TuningConstants.MAGIC_NULL_VALUE);
        this.heartbeatSubscriber = this.networkTable.getIntegerSubscriber("v.heartbeat", 0);

        this.ds = provider.getDriverStation();

        this.atXOffset = null;
        this.atYOffset = null;
        this.atZOffset = null;
        this.atYaw = null;
        this.atPitch = null;
        this.atRoll = null;
        this.atId = null;

        this.missedHeartbeats = 0;
        this.prevHeartbeat = 0L;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.atXOffset = this.atXOffsetSubscriber.get();
        this.atYOffset = this.atYOffsetSubscriber.get();
        this.atZOffset = this.atZOffsetSubscriber.get();
        this.atYaw = this.atYawSubscriber.get();
        this.atPitch = this.atPitchSubscriber.get();
        this.atRoll = this.atRollSubscriber.get();
        this.atId = (int)this.atIdSubscriber.get();

        long newHeartbeat = this.heartbeatSubscriber.get();
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

        // reset if we couldn't find the april tag
        if (missedHeartbeatExceedsThreshold || this.atXOffset == TuningConstants.MAGIC_NULL_VALUE || this.atYOffset == TuningConstants.MAGIC_NULL_VALUE || this.atZOffset == TuningConstants.MAGIC_NULL_VALUE)
        {
            this.atXOffset = null;
            this.atYOffset = null;
            this.atZOffset = null;
            this.atYaw = null;
            this.atPitch = null;
            this.atRoll = null;
            this.atId = null;
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
    public void update()
    {
        boolean enableVision = !this.driver.getDigital(DigitalOperation.VisionForceDisable);
        boolean enableVideoStream = false; //!this.driver.getDigital(DigitalOperation.VisionDisableStream);
        boolean enableAprilTagProcessing = this.driver.getDigital(DigitalOperation.VisionEnableAprilTagProcessing);

        Optional<Alliance> alliance = this.ds.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

        int desiredTarget = 0;
        double visionProcessingMode = 0.0;
        if (enableVision)
        {
            if (enableAprilTagProcessing)
            {
                visionProcessingMode = 1.0;
                if (isRed)
                {
                    desiredTarget = TuningConstants.APRILTAG_RED_SPEAKER_CENTER_ID;
                }
                else
                {
                    desiredTarget = TuningConstants.APRILTAG_BLUE_SPEAKER_CENTER_ID;
                }
            }
        }

        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, enableVideoStream);
        this.logger.logNumber(LoggingKey.OffboardVisionProcessingMode, visionProcessingMode);
        this.logger.logNumber(LoggingKey.OffboardVisionDesiredTarget, desiredTarget);
    }

    @Override
    public void stop()
    {
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, false);
        this.logger.logNumber(LoggingKey.OffboardVisionProcessingMode, 0.0);
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
}
