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
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-15.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-30.0, 0.0, 180.0, 0.0)),
            "goBackwards30in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-6.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-12.0, 0.0, 180.0, 0.0)),
            "goBackwards1ft");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-6.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-12.0, 0.0, 180.0, 0.0)),
            "goBackwards15in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(9.0, 16.0, 0.0, 0.0),
                new PathPlannerWaypoint(18.0, 32.0, 0.0, 0.0)),
            "goLeft32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(9.0, -16.0, 0.0, 0.0),
                new PathPlannerWaypoint(18.0, -32.0, 0.0, 0.0)),
            "goRight32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 11.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 22.0, 90.0, 0.0)),
            "goLeft22in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(0.0, -11.0, 270.0, 0.0),
                new PathPlannerWaypoint(0.0, -22.0, 270.0, 0.0)),
            "goRight22in");
    }

    public static void generateTrajectories(boolean isRed, TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        //Red is positive
        //Blue is negative

        //Y field length: 323in
        //X field length: 653in

        double framePreremetere = 34; //With bumpers
        double halfFramePreremetere = framePreremetere / 2.0;

        Point2d P1 = new Point2d(getXPosition(isRed, 250 + halfFramePreremetere), 23 + halfFramePreremetere); // Need to add starting position
        Point2d P2 = new Point2d(getXPosition(isRed, 312 - 0) , 177); //-30 degrees heading
        Point2d P3 = new Point2d(getXPosition(isRed, 250.5 + halfFramePreremetere), 306 - halfFramePreremetere);
        Point2d P4 = new Point2d(getXPosition(isRed, 288 - halfFramePreremetere), 239 - halfFramePreremetere);

        Point2d P5 = new Point2d(getXPosition(isRed, 212), 162);
        Point2d P5M = new Point2d(getXPosition(isRed, P5.x + 20), 162);

        Point2d P6 = new Point2d(getXPosition(isRed, 212), 219);
        Point2d P6M = new Point2d(getXPosition(isRed, P6.x + 20), 219);

        Point2d P7 = new Point2d(getXPosition(isRed, 212), 276);
        Point2d P7M = new Point2d(getXPosition(isRed, P7.x + 20), 160.888409);

        Point2d P8 = new Point2d(getXPosition(isRed, 0), 29.64);
        Point2d P8M = new Point2d(getXPosition(isRed, P8.x + 20), 29.64);

        Point2d P9 = new Point2d(getXPosition(isRed, 0), 95.64);
        Point2d P9M = new Point2d(getXPosition(isRed, P9.x + 20), 95.64);

        Point2d P10 = new Point2d(getXPosition(isRed, 0), 161.64);
        Point2d P10M = new Point2d(getXPosition(isRed, P10.x + 20), 161.64);
        
        Point2d P11 = new Point2d(getXPosition(isRed, 0), 227.64);
        Point2d P11M = new Point2d(getXPosition(isRed, P11.x + 20), 227.64);

        Point2d P12 = new Point2d(getXPosition(isRed, 0), 293.64);
        Point2d P12M = new Point2d(getXPosition(isRed, P12.x + 20), 293.64);

        //ToDO fix 13 14
        Point2d P13 = new Point2d(getXPosition(isRed, 0), 93.154754 - halfFramePreremetere);
        Point2d P14 = new Point2d(getXPosition(isRed, 0), 231.777 + halfFramePreremetere);
        //ToDo : add p15, p16, p17
        Point2d P15 = new Point2d(0,0);//Might use


        Point2d P16 = new Point2d(getXPosition(isRed, 172.955),  93.154754 - halfFramePreremetere);
        Point2d P17 = new Point2d(getXPosition(isRed, 172.955),  231.777 + halfFramePreremetere);

        Point2d P18 = new Point2d(getXPosition(isRed, 115 + 21 + halfFramePreremetere), 162);


        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P3, getOrientationOrHeading(isRed, 45), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P7M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P7, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P5M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P5, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180))),
                "P3toP5");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P2, getOrientationOrHeading(isRed, 45), getOrientationOrHeading(isRed, 180)), // fix orientation
                new PathPlannerWaypoint(P5M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P5, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P5M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P16, getOrientationOrHeading(isRed, 45), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P8M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P8, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180))),
                "P2toP8");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P4, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P18, getOrientationOrHeading(isRed, 45), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P11M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P11, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180))),
                "P4toP11");


        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P1, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P8M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P8, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P9M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P9, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P11M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P11, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P12M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P12, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180))),
            "P1toP12");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P3, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P7M, getOrientationOrHeading(isRed, 45), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P7, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                //Stop for Note Pickup
                new PathPlannerWaypoint(P7, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P12M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P12, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(P11M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P11, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(P10M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(P9M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P9, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(P8M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P8, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180))),
                //Possible Note Pickup
                "P3toP8");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(P2, getOrientationOrHeading(isRed, 315), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6M, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P14, getOrientationOrHeading(isRed, 315), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10M, getOrientationOrHeading(isRed, 45), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P10, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180))),
                "P2toP10");  
    }

    

    public static double getXPosition(boolean isRed, double position)
    {
        if(isRed)
        {
            return position;
        }
        else
        {
            return position * -1.0;
        }
    }

    //TODO can getOrientationorHeading() go in AutonLocManager?

    public static double getOrientationOrHeading(boolean isRed, double orientationOrHeading)
    {
        if(isRed)
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
// Big Yush Was Also Here