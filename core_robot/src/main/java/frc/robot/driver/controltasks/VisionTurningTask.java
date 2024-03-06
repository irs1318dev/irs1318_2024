package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class VisionTurningTask extends PIDTurnTaskBase
{
    private static final DigitalOperation[] PossibleVisionOperations =
    {
        DigitalOperation.VisionFindAnyAprilTagFront,
        DigitalOperation.VisionFindAnyAprilTagRear,
        DigitalOperation.VisionFindSpeakerAprilTagFront,
        DigitalOperation.VisionFindSpeakerAprilTagRear,
        DigitalOperation.VisionFindAmpAprilTagFront,
        DigitalOperation.VisionFindAmpAprilTagRear,
        DigitalOperation.VisionFindStageAprilTagsFront,
        DigitalOperation.VisionFindStageAprilTagsRear,
    };

    public enum TurnType
    {
        None,
        AprilTagCentering,
        AprilTagParallelizing,
    }

    protected final TurnType rotateType;
    protected final DigitalOperation visionOperation;

    protected OffboardVisionManager visionManager;

    /**
    * Initializes a new VisionTurningTask
     * @param rotateType what type of turning to do
    */
    public VisionTurningTask(TurnType rotateType, DigitalOperation visionOperation)
    {
        this(true, rotateType, false, visionOperation);
    }

    /**
    * Initializes a new VisionTurningTask
     * @param rotateType what type of turning to do
     * @param bestEffort whether to end (true) or cancel (false, default) when we cannot see the game piece or vision target (for sequential tasks, whether to continue on or not)
    */
    public VisionTurningTask(TurnType rotateType, boolean bestEffort, DigitalOperation visionOperation)
    {
        this(true, rotateType, bestEffort, visionOperation);
    }

    /**
     * Initializes a new VisionTurningTask
     * @param useTime whether to make sure we are centered for a second or not
     * @param rotateType what type of turning to do
     * @param bestEffort whether to end (true) or cancel (false, default) when we cannot see the game piece or vision target (for sequential tasks, whether to continue on or not)
     */
    public VisionTurningTask(boolean useTime, TurnType rotateType, boolean bestEffort, DigitalOperation visionOperation)
    {
        super(useTime, bestEffort);
        this.rotateType = rotateType;
        this.visionOperation = visionOperation;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);

        for (DigitalOperation op : VisionTurningTask.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, op == this.visionOperation);
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        for (DigitalOperation op : VisionTurningTask.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, false);
        }
    }

    @Override
    protected Double getHorizontalAngle()
    {
        Double angle;
        switch (this.rotateType)
        {
            case AprilTagParallelizing:
                // turn to match the yaw, so we are lined up parallel to the tag
                angle = this.visionManager.getAprilTagYaw();
                if (angle != null)
                {
                    angle = -1.0 * angle;
                }

                break;

            case AprilTagCentering:
                // Note: we want to point toward the AprilTag, not match its yaw (make ourselves parallel to it), so we can use the fact that tan(angle) = opposite / adjacent
                Double xOffset = this.visionManager.getAprilTagXOffset();
                Double yOffset = this.visionManager.getAprilTagYOffset();
                if (xOffset == null || yOffset == null)
                {
                    angle = null;
                }
                else
                {
                    angle = -Helpers.atan2d(yOffset, xOffset);
                }

                break;

            default:
            case None:
                angle = 0.0;
                break;
        }

        return angle;
    }
}