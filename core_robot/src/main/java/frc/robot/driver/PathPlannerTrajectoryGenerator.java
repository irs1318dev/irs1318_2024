package frc.robot.driver;

import frc.lib.driver.TrajectoryManager;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.lib.robotprovider.Point2d;
import frc.robot.AutonLocManager;
import frc.robot.TuningConstants;

import frc.robot.AutonLocManager;

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
        AutonLocManager LocManager = new AutonLocManager(isRed);

        //Verified Red
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LocManager.P3, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P7M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P7, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P6M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P6, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P5M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P5, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
                isRed?"P3toP5Red":"P3toP5Blue");

        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LocManager.P2, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 210)), // fix orientation
                new PathPlannerWaypoint(LocManager.P5M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P5, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P5M, getOrientationOrHeading(isRed, 255), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P16, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P8M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P8, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
                "P2toP8");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LocManager.P4, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P6M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P6, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P18, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P10M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P10, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P11M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P11, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
                isRed ? "P4toP11Red" : "P4toP11Blue");


        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LocManager.P1, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P8M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P8, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P9M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P9, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P10M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P10, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P11M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P11, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P12M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P12, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
            isRed ? "P1toP12Red" : "P1toP12Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LocManager.P3, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P7M, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P7, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                //Stop for Note Pickup
                new PathPlannerWaypoint(LocManager.P12M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P12, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(LocManager.P11M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P11, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(LocManager.P10M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P10, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(LocManager.P9M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P9, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(LocManager.P8M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P8, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
                //Possible Note Pickup
                isRed? "P3toP8Red" : "P3toP8Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LocManager.P2, getOrientationOrHeading(isRed, 135), getOrientationOrHeading(isRed, 210)),
                new PathPlannerWaypoint(LocManager.P6M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P6, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P14, getOrientationOrHeading(isRed, 135), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P10M, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P10, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180))),
                isRed? "P2toP10Red" : "P2toP10Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
        //         TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(getXPosition(isRed, 0),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
        //         new PathPlannerWaypoint(getXPosition(isRed, 0),80, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
        //         new PathPlannerWaypoint(getXPosition(isRed,-80),80, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
        //         new PathPlannerWaypoint(getXPosition(isRed,-80),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180)),
        //         new PathPlannerWaypoint(getXPosition(isRed,0),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 180))),
        //         isRed? "P3toP16Red" : "P3toP16Blue");
                
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LocManager.P3, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 0)),
                new PathPlannerWaypoint(LocManager.P17, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 0))
                //new PathPlannerWaypoint(getXPosition(isRed,-80),80, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,-80),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,0),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0))
                ),
                isRed? "AyushTestRed" : "AyushTestBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(

            
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,

                /* 
                new PathPlannerWaypoint(P3, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)), 
                new PathPlannerWaypoint(P20, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P6M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(P4, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 0))//end facing left
*/

                new PathPlannerWaypoint(LocManager.P12, getOrientationOrHeading(isRed, 270), getOrientationOrHeading(isRed, 180)), 
                new PathPlannerWaypoint(LocManager.P11, getOrientationOrHeading(isRed, 270), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P10, getOrientationOrHeading(isRed, 270), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P9, getOrientationOrHeading(isRed, 270), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P8, getOrientationOrHeading(isRed, 270), getOrientationOrHeading(isRed, 180))
                //new PathPlannerWaypoint(P4, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 0))//end facing left

                
                //new PathPlannerWaypoint(P1, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 0))//end facing us
                //new PathPlannerWaypoint(getXPosition(isRed,-80),80, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,-80),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,0),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0))

                //new PathPlannerWaypoint(P3, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 180)), 
                //new PathPlannerWaypoint(P6M, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 90)),
                //new PathPlannerWaypoint(P7, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed, 270))
                ),
                isRed? "JamieAndAyushPathRed" : "JamieAndAyushPathBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(LocManager.P4, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P5, getOrientationOrHeading(isRed, 90), getOrientationOrHeading(isRed, 90)),
                new PathPlannerWaypoint(LocManager.P6, getOrientationOrHeading(isRed, 90), getOrientationOrHeading(isRed, 90)),
                new PathPlannerWaypoint(LocManager.P7, getOrientationOrHeading(isRed, 90), getOrientationOrHeading(isRed, 180)),
                new PathPlannerWaypoint(LocManager.P12, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(LocManager.P20, getOrientationOrHeading(isRed, 135), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(LocManager.P14, getOrientationOrHeading(isRed, 135), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(LocManager.P11, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(LocManager.P14, getOrientationOrHeading(isRed, 225), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(LocManager.P10, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(LocManager.P18, getOrientationOrHeading(isRed, 45), getOrientationOrHeading(isRed,180)),
                new PathPlannerWaypoint(LocManager.P20, getOrientationOrHeading(isRed, 180), getOrientationOrHeading(isRed,180))

                //new PathPlannerWaypoint(getXPosition(isRed,-80),80, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,-80),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0)),
                //new PathPlannerWaypoint(getXPosition(isRed,0),0, getOrientationOrHeading(isRed, 0), getOrientationOrHeading(isRed, 0))
                ),
                isRed? "SevenNoteAutoRed" : "SevenNoteAutoBlue");
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
            return 180.0 - orientationOrHeading;
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