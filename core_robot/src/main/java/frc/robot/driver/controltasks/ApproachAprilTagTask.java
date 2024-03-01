package frc.robot.driver.controltasks;

import frc.lib.driver.TrajectoryManager;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

//SequentialTask.Sequence(
//    ArmGraphTask(GroundIntakePosition),
//    ApproachTrapTask(),
//    ArmGraphTask(ClimbPosition),
//    FollowPathTask(XinchesForward), // Engage
//    WinchTask(timeout),
//    ArmGraphTask(TrapScorePosition),
//    IntakeControlTask(out, 2s)
//)
public class ApproachAprilTagTask extends DecisionSequentialTask
{
    private static final DigitalOperation[] PossibleVisionOperations =
    {
        DigitalOperation.VisionFindAnyAprilTagFront,
        DigitalOperation.VisionFindAnyAprilTagRear,
        DigitalOperation.VisionFindSpeakerAprilTagFront,
        DigitalOperation.VisionFindSpeakerAprilTagRear,
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
        this(-72.0, 0.0, DigitalOperation.VisionFindStageAprilTagsFront);
    }

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

                double xGoal = Helpers.cosd(tagYawOffset) * this.xOffset + Helpers.sind(tagYawOffset) * this.yOffset + tagXOffset;
                double yGoal = Helpers.cosd(tagYawOffset) * this.yOffset - Helpers.sind(tagYawOffset) * this.xOffset + tagYOffset;
                double angleGoal = tagYawOffset;

                // generate the path
                this.trajectoryManager.addTrajectory(
                    "climberApproachMovement", 
                    pathPlanner.buildTrajectory(
                        TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                        TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                        TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                        TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                        new PathPlannerWaypoint(0, 0, 0, 0),
                        new PathPlannerWaypoint(xGoal, yGoal, angleGoal, angleGoal)));
                this.AppendTask(new FollowPathTask("climberApproachMovement"));
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