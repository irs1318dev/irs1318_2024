package frc.robot.driver;

import frc.lib.driver.TrajectoryManager;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.lib.robotprovider.Point2d;
import frc.robot.TuningConstants;

public class PathPlannerTrajectoryGenerator
{
    public static void generateTrajectories(TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        PathPlannerTrajectoryGenerator.generateTrajectories(false, trajectoryManager, pathPlanner);
        PathPlannerTrajectoryGenerator.generateTrajectories(true, trajectoryManager, pathPlanner);

        // ------------------------------- Macro paths --------------------------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                0.0,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-30.0, 0.0, 180.0, 0.0)),
                "goBackwards30in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                0.0,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-12.0, 0.0, 180.0, 0.0)),
                "goBackwards1ft");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                0.0,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-12.0, 0.0, 180.0, 0.0)),
                "goBackwards15in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                0.0,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(18.0, 32.0, 0.0, 0.0)),
                "goLeft32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                0.0,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(18.0, -32.0, 0.0, 0.0)),
                "goRight32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                0.0,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 22.0, 90.0, 0.0)),
                "goLeft22in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                0.0,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(0.0, -22.0, 270.0, 0.0)),
                "goRight22in");
    }

    static double framePreremetere = 34; //With bumpers
    static double halfFramePreremetere = framePreremetere / 2;

    static Point2d P1 = new Point2d(getXPosition(324.173750 - halfFramePreremetere), 253.611250 - halfFramePreremetere);
    static Point2d P2 = new Point2d(getXPosition(299.199673 - 0) , 78.285726 - 0); //X and Y need to be figured out and stubstracted
    static Point2d P3 = new Point2d(getXPosition(250.5 + halfFramePreremetere), 324.0 + halfFramePreremetere);
    static Point2d P4 = new Point2d(getXPosition(288.432368 - halfFramePreremetere), 116.473567 + halfFramePreremetere);
    static Point2d P5 = new Point2d(getXPosition(212.6), 46.888409);
    static Point2d P6 = new Point2d(getXPosition(212.6), 103.888409);
    static Point2d P7 = new Point2d(getXPosition(212.6), 160.888409);
    static Point2d P8 = new Point2d(getXPosition(0), 29.64);
    static Point2d P9 = new Point2d(getXPosition(0), 95.64);
    static Point2d P10 = new Point2d(getXPosition(0), 161.64);
    static Point2d P11 = new Point2d(getXPosition(0), 227.64);
    static Point2d P12 = new Point2d(getXPosition(0), 293.64);

    public static void generateTrajectories(boolean isRed, TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {

        //Need to fix headding and orientation for paths, not usable
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                0.0,
                new PathPlannerWaypoint(P2, getOrientationOrHeading(0), getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(P5, getOrientationOrHeading(0), getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(P8, getOrientationOrHeading(0), getOrientationOrHeading(0.0))),
                "P2toP8");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                0.0,
                new PathPlannerWaypoint(P3, getOrientationOrHeading(0), getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(P7, getOrientationOrHeading(0), getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(P12, getOrientationOrHeading(0), getOrientationOrHeading(0.0))),
                "P3toP12");
    }

    
    /* 
    public static double getXPosition(double position)
    {
        if(TuningConstants.IS_RED)
        {
            return position;
        }
        else
        {
            return position * -1.0;
        }
    }
    */

    public static double getOrientationOrHeading(double orientationOrHeading)
    {
        if(TuningConstants.IS_RED)
        {
            return orientationOrHeading;
        }
        else
        {
            return orientationOrHeading - 180.0;
        }
    }

    private static void addTrajectory(TrajectoryManager trajectoryManager, ITrajectory trajectory, String name)
    {
        try
        {
            trajectoryManager.addTrajectory(name, trajectory);
        }
        catch (Exception ex)
        {
            System.err.println("Encountered exception generating path " + name + ": " + ex.toString());
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw ex;
            }
        }
    }
}