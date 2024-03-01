package frc.robot.driver.controltasks;

import frc.lib.driver.TrajectoryManager;
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
public class ApproachTrapTask extends DecisionSequentialTask
{
    private OffboardVisionManager vision;
    private IRobotProvider provider;
    private TrajectoryManager trajectoryManager;

    private double xOffset;
    private double yOffset;
    private double yawOffset;
    private int noAprilTags;

    public enum State
    {
        ReadAprilTag,
        ApproachAprilTag
    }

    private State state;

    @Override
    public void begin()
    {
        super.begin();

        this.vision = this.getInjector().getInstance(OffboardVisionManager.class);     
        this.provider = this.getInjector().getInstance(IRobotProvider.class);
        this.trajectoryManager = this.getInjector().getInstance(TrajectoryManager.class);

        this.setDigitalOperationState(DigitalOperation.VisionFindStageAprilTagsFront, true);
        this.state = State.ReadAprilTag;
    }

    @Override
    public void update()
    {
        if (this.state == State.ReadAprilTag)
        {
            if (this.vision.getAprilTagId() != null)
            {
                IPathPlanner pathPlanner = this.provider.getPathPlanner();
                this.xOffset = vision.getAprilTagXOffset();
                this.yOffset = vision.getAprilTagYOffset();
                this.yawOffset = vision.getAprilTagYaw();

                // generate the path
                this.trajectoryManager.addTrajectory(
                    "climberApproachMovement", 
                    pathPlanner.buildTrajectory(
                        TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                        TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                        TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                        TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                        new PathPlannerWaypoint(0, 0, 0, 0),
                        new PathPlannerWaypoint(xOffset, yOffset, yawOffset, yawOffset)));
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

        this.setDigitalOperationState(DigitalOperation.VisionFindStageAprilTagsFront, false);
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