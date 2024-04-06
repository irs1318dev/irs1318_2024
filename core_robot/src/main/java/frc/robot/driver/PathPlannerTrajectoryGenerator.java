package frc.robot.driver;

import frc.lib.driver.TrajectoryManager;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.PathPlannerRotationTarget;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.robot.AutonLocManager;
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
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(15.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(30.0, 0.0, 0.0, 0.0)),
            "goForwards30in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(30.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(45.0, 0.0, 0.0, 0.0)),
            "goForwards45in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(20.0, 0.0, 0.0, 0.0)),
            "DriveForward20inTrap");
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
        AutonLocManager locManager = new AutonLocManager(isRed);

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
        //         TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(locManager.P3, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
        //         new PathPlannerWaypoint(locManager.P3, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(135))),
        //         isRed?"P3RotateToShootRed":"P3RotateToShootBlue");

        //  --------------------------> SHOOT TAXI PATHS <-----------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P4toP6STRed" : "P4toP6STBlue");
        

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2, locManager.getOrientationOrHeading(236.7), locManager.getOrientationOrHeading(236.7)),
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(236.7), locManager.getOrientationOrHeading(236.7))),
                isRed? "P2toP19STRed" : "P2toP19STBlue");

        // ----------------------> CENTER SUBWOOFER PATHS <-----------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P4toP6CSRed" : "P4toP6CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180))),
                isRed? "P6toP4CSRed" : "P6toP4CSBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P4toP7CSRed" : "P4toP7CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(315), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(315), locManager.getOrientationOrHeading(180))),
                isRed? "P7toP4CSRed" : "P7toP4CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P4toP5CSRed" : "P4toP5CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(180))),
                isRed? "P5toP4CSRed" : "P5toP4CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
               TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
               TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
               TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
               TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
               new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
               new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180))),
               isRed? "P6toP6MCSRed" : "P6toP6MCSBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(90), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P6MtoP7CSRed" : "P6MtoP7CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(295), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(295), locManager.getOrientationOrHeading(180))),
                isRed? "P7toP6MCSRed" : "P7toP6MCSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P6MtoP5CSRed" : "P6MtoP5CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(65), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(65), locManager.getOrientationOrHeading(180))),
                isRed? "P5toP6MCSRed" : "P5toP6MCSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P20, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(180))),
                isRed? "P6MtoP10CSRed" : "P6MtoP10CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P20, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P4toP10CSRed" : "P4toP10CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P20, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(180))),
                isRed? "P10toP20CSRed" : "P10toP20CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P9M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P4toP8CSRed" : "P4toP8CSBlue");

        // ----------------------> AMP SUB PATHS <-----------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(123.3)),
                new PathPlannerWaypoint(locManager.P7M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P2AtoP7ASUBRed" : "P2AtoP7ASUBBlue");   

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(123.3), 0.6),
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(315), locManager.getOrientationOrHeading(123.3))),
                isRed? "P7toP2AASUBRed" : "P7toP2AASUBBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(123.3)),
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(180), 0.5),
                new PathPlannerWaypoint(locManager.P12M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P2AtoP12ASUBRed" : "P2AtoP12ASUBBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(123.3), 0.7),
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(315), locManager.getOrientationOrHeading(123.3))),
                isRed? "P12toP2ASUBRed" : "P12toP2ASUBBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(123.3)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P2AtoP11ASUBRed" : "P2AtoP11ASUBBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(123.3), 0.7),
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(315), locManager.getOrientationOrHeading(123.3))),
                isRed? "P11toP2ASUBRed" : "P11toP2ASUBBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P12M, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P12toP11ASUBRed" : "P12toP11ASUBBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(123.3)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11M, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P2AtoP11ASUBRed" : "P2AtoP11ASUBBlue");
                
        // ----------------------> SOURCE SUB PATHS <-----------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2, locManager.getOrientationOrHeading(240), locManager.getOrientationOrHeading(236.7)),
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(236.7), 0.2),
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(200)),
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(180), 0.6),
                new PathPlannerWaypoint(locManager.P8M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P2toP8SSRed" : "P2toP8SSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(64.3), locManager.getOrientationOrHeading(236.7)),
                new PathPlannerWaypoint(locManager.P2, locManager.getOrientationOrHeading(64.3), locManager.getOrientationOrHeading(236.7))),
                isRed? "P8toP2SSRed" : "P8toP2SSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P8M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P9M, locManager.getOrientationOrHeading(90), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180))),
                isRed? "P8toP9SSRed" : "P8toP9SSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(200)),
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(64.3), locManager.getOrientationOrHeading(236.7)),
                new PathPlannerWaypoint(locManager.P2, locManager.getOrientationOrHeading(64.3), locManager.getOrientationOrHeading(236.7))),
                isRed? "P9toP2SSRed" : "P9toP2SSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(45.0), locManager.getOrientationOrHeading(205.0)),
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(225.0), 0.5),
                new PathPlannerWaypoint(locManager.P19S, locManager.getOrientationOrHeading(45.0), locManager.getOrientationOrHeading(225.0))),
                isRed? "P8toP19SSSRed" : "P8toP19SSSBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P19S, locManager.getOrientationOrHeading(225.0), locManager.getOrientationOrHeading(225.0)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(205.0)),
                new PathPlannerWaypoint(locManager.P9M, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
                isRed? "P19StoP9SSRed" : "P19StoP9SSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(315.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(45.0), locManager.getOrientationOrHeading(205.0)),
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(225.0), 0.5),
                new PathPlannerWaypoint(locManager.P19S, locManager.getOrientationOrHeading(45.0), locManager.getOrientationOrHeading(225.0))),
                isRed? "P9toP19SSSRed" : "P9toP19SSSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P19S, locManager.getOrientationOrHeading(135.0), locManager.getOrientationOrHeading(225.0)),
                new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(135.0), locManager.getOrientationOrHeading(205.0)),
                new PathPlannerWaypoint(locManager.P10M, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
                isRed? "P19StoP10SSRed" : "P19StoP10SSBlue");
        
        
        // ----------------------> SNEAK PATHS <-----------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(135.0), locManager.getOrientationOrHeading(123.3)),
                new PathPlannerWaypoint(locManager.PSNK1, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.PSNK2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P12M, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
                isRed? "P2Ato12SNKRed" : "P2Ato12SNKBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(165.0), 0.7),
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(165.0))),
                isRed? "P12toP17SNKRed" : "P12toP17SNKBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(165.0)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(225.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P11M, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
                isRed? "P17to11SNKRed" : "P17to11SNKBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(45.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(45.0), locManager.getOrientationOrHeading(155.0)),
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(165.0), 0.5),
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(165.0))),
                isRed? "P11toP17SNKRed" : "P11toP17SNKBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(165.0)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P10M, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
                isRed? "P17to10SNKRed" : "P17to10SNKBlue");


        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P12M, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P11M, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
                isRed? "P12toP11SNKRed" : "P12toP11SNKBlue");

        // ----------------------> AMP START PATHS <-----------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P3, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P3toP7ASRed" : "P3toP7ASBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P7toP6MASRed" : "P7toP6MASBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P6MtoP6ASRed" : "P6MtoP6ASBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P12M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P6toP12ASRed" : "P6toP12ASBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180))),
                isRed? "P12toP6ASRed" : "P12toP6ASBlue");


        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P1, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P8M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P9M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P12M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
            isRed ? "P1toP12Red" : "P1toP12Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P3, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7M, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                //Stop for Note Pickup
                new PathPlannerWaypoint(locManager.P12M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(locManager.P11M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(locManager.P10M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(locManager.P9M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                //Possible Note Pickup
                new PathPlannerWaypoint(locManager.P8M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                //Possible Note Pickup
                isRed? "P3toP8Red" : "P3toP8Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(210)),
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10M, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P2toP10Red" : "P2toP10Blue");

        // addTrajectory(
        //     trajectoryManager,
        //     pathPlanner.buildTrajectory(
        //         TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
        //         TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
        //         TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
        //         TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
        //         new PathPlannerWaypoint(getXPosition(isRed, 0),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
        //         new PathPlannerWaypoint(getXPosition(isRed, 0),80, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
        //         new PathPlannerWaypoint(getXPosition(isRed,-80),80, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
        //         new PathPlannerWaypoint(getXPosition(isRed,-80),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
        //         new PathPlannerWaypoint(getXPosition(isRed,0),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180))),
        //         isRed? "P3toP16Red" : "P3toP16Blue");
                
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P3, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(0)),
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(0))
                //new PathPlannerWaypoint(getXPosition(isRed,-80),80, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0)),
                //new PathPlannerWaypoint(getXPosition(isRed,-80),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0)),
                //new PathPlannerWaypoint(getXPosition(isRed,0),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0))
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
                new PathPlannerWaypoint(P3, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)), 
                new PathPlannerWaypoint(P20, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(0))//end facing left
*/

                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(180)), 
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(180))
                //new PathPlannerWaypoint(P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(0))//end facing left

                
                //new PathPlannerWaypoint(P1, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(0))//end facing us
                //new PathPlannerWaypoint(getXPosition(isRed,-80),80, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0)),
                //new PathPlannerWaypoint(getXPosition(isRed,-80),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0)),
                //new PathPlannerWaypoint(getXPosition(isRed,0),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0))

                //new PathPlannerWaypoint(P3, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)), 
                //new PathPlannerWaypoint(P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(90)),
                //new PathPlannerWaypoint(P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(270))
                ),
                isRed? "JamieAndAyushPathRed" : "JamieAndAyushPathBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(90), locManager.getOrientationOrHeading(90)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(90), locManager.getOrientationOrHeading(90)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(90), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P20, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P20, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))

                //new PathPlannerWaypoint(getXPosition(isRed,-80),80, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0)),
                //new PathPlannerWaypoint(getXPosition(isRed,-80),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0)),
                //new PathPlannerWaypoint(getXPosition(isRed,0),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0))
                ),
                isRed? "SevenNoteAutoRed" : "SevenNoteAutoBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P1, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P16, locManager.getOrientationOrHeading(90), locManager.getOrientationOrHeading(90)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(90), locManager.getOrientationOrHeading(90)),
                new PathPlannerWaypoint(locManager.P8M, locManager.getOrientationOrHeading(90), locManager.getOrientationOrHeading(180))

                //new PathPlannerWaypoint(getXPosition(isRed,-80),80, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0)),
                //new PathPlannerWaypoint(getXPosition(isRed,-80),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0)),
                //new PathPlannerWaypoint(getXPosition(isRed,0),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0))
                ),
                isRed? "P1toP8MRed" : "P1toP8MBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P8M, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(90), locManager.getOrientationOrHeading(90))

                //new PathPlannerWaypoint(getXPosition(isRed,-80),80, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0)),
                //new PathPlannerWaypoint(getXPosition(isRed,-80),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0)),
                //new PathPlannerWaypoint(getXPosition(isRed,0),0, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(0))
                ),
                isRed? "P8MtoP8Red" : "P8MtoP8Blue");
    }

    private static void addTrajectory(TrajectoryManager trajectoryManager, ITrajectory trajectory, String name)
    {
        // ExceptionHelpers.Assert(trajectory != null, "Adding null trajectory '%s'!", name);
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