package frc.robot.driver.controltasks;

import java.util.List;

import frc.lib.driver.TrajectoryManager;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.controltasks.FollowPathTask.Type;
import frc.robot.mechanisms.OffboardVisionManager;

public class ApproachAprilTagTask extends DecisionSequentialTask
{
    private static final String PATH_NAME = "ApproachAprilTagTaskPath";
    private static final List<DigitalOperation> PossibleFrontVisionOperations =
        List.of(
            DigitalOperation.VisionFindAnyAprilTagFront,
            DigitalOperation.VisionFindSpeakerAprilTagFront,
            DigitalOperation.VisionFindAmpAprilTagFront,
            DigitalOperation.VisionFindStageAprilTagsFront);

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

    private final double xOffset;
    private final double yOffset;
    private final DigitalOperation visionOperation;

    private OffboardVisionManager vision;
    private IRobotProvider provider;
    private TrajectoryManager trajectoryManager;

    private int noAprilTags;

    public enum State
    {
        ReadAprilTag,
        ApproachAprilTag
    }

    private State state;

    public ApproachAprilTagTask()
    {
        this(72.0, 0.0, DigitalOperation.VisionFindStageAprilTagsFront);
    }

    /**
     * Initializes an instance of the ApproachAprilTagTask class
     * @param xOffset the distance the robot should end up in front of the tag
     * @param yOFfset the distance the robot should end up to the left of the tag
     * @param visionOperation the vision operation to use to find the tag
     */
    public ApproachAprilTagTask(double xOffset, double yOFfset, DigitalOperation visionOperation)
    {
        if (TuningConstants.THROW_EXCEPTIONS)
        {
            // if we are cool with throwing exceptions (testing), check if toPerform is in
            // the possibleOperations set and throw an exception if it is not
            boolean containsToPerform = false;
            for (DigitalOperation op : ApproachAprilTagTask.PossibleVisionOperations)
            {
                if (op == visionOperation)
                {
                    containsToPerform = true;
                    break;
                }
            }

            ExceptionHelpers.Assert(containsToPerform, visionOperation.toString() + " not contained in the set of possible vision operations");
        }

        this.xOffset = xOffset;
        this.yOffset = yOFfset;
        this.visionOperation = visionOperation;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.vision = this.getInjector().getInstance(OffboardVisionManager.class);     
        this.provider = this.getInjector().getInstance(IRobotProvider.class);
        this.trajectoryManager = this.getInjector().getInstance(TrajectoryManager.class);

        for (DigitalOperation op : ApproachAprilTagTask.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, op == this.visionOperation);
        }

        this.state = State.ReadAprilTag;
    }

    @Override
    public void update()
    {
        if (this.state == State.ReadAprilTag)
        {
            for (DigitalOperation op : ApproachAprilTagTask.PossibleVisionOperations)
            {
                this.setDigitalOperationState(op, op == this.visionOperation);
            }

            if (this.vision.getAprilTagId() != null)
            {
                IPathPlanner pathPlanner = this.provider.getPathPlanner();
                double tagXOffset = vision.getAprilTagXOffset();
                double tagYOffset = vision.getAprilTagYOffset();
                double tagYawOffset = vision.getAprilTagYaw();

                double xGoal = tagXOffset - Helpers.cosd(tagYawOffset) * this.xOffset - Helpers.sind(tagYawOffset) * this.yOffset;
                double yGoal = tagYOffset + Helpers.cosd(tagYawOffset) * this.yOffset - Helpers.sind(tagYawOffset) * this.xOffset;
                double angleGoal = tagYawOffset;

                boolean backwards = !ApproachAprilTagTask.PossibleFrontVisionOperations.contains(this.visionOperation);
                if (backwards)
                {
                    xGoal *= -1.0;
                    yGoal *= -1.0;
                }

                double tangent = Helpers.atan2d(yGoal, angleGoal);

                // generate the path
                this.trajectoryManager.addTrajectory(
                    ApproachAprilTagTask.PATH_NAME, 
                    pathPlanner.buildTrajectory(
                        TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                        TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                        TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                        TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                        new PathPlannerWaypoint(0, 0, tangent, 0),
                        new PathPlannerWaypoint(xGoal, yGoal, tangent, angleGoal)));
                this.AppendTask(new FollowPathTask(ApproachAprilTagTask.PATH_NAME, Type.RobotRelativeFromCurrentPose));
                this.state = State.ApproachAprilTag;
            }
            else
            {
                noAprilTags++;
            }
        }
        else // if (this.state == State.ApproachAprilTag)
        {
            super.update();
        }
    }

    @Override
    public void end()
    {
        if (this.state != State.ReadAprilTag)
        {
            super.end();
        }

        for (DigitalOperation op : ApproachAprilTagTask.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, false);
        }
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.state != State.ApproachAprilTag)
        {
            return false;
        }

        return super.hasCompleted();
    }

    @Override
    public boolean shouldCancel()
    {
        if (this.state == State.ReadAprilTag)
        {
            return this.noAprilTags > 20;
        }

        return super.shouldCancel();
    }
}