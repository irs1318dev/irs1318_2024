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
                new PathPlannerWaypoint(55.0, 0.0, 0.0, 0.0)),
            "goForwards55in");
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
                isRed? "P4toP6Red" : "P4toP6Blue");
        

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(225)),
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(225))),
                isRed? "P2toP19Red" : "P2toP19Blue");

        // ----------------------> CENTER SUBWOOFER PATHS <-----------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P4toP6CSRed" : "P4toP6CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180))),
                isRed? "P6toP4CSRed" : "P6toP4CSBlue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P4toP7CSRed" : "P4toP7CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(315), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(315), locManager.getOrientationOrHeading(180))),
                isRed? "P7toP4CSRed" : "P7toP4CSBlue");

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
                isRed? "P6toP12CSRed" : "P6toP12CSBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(315), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(315), locManager.getOrientationOrHeading(180))),
                isRed? "P5toP4CSRed" : "P5toP4CSBlue");

        // ----------------------> AMP SUB PATHS <-----------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(120)),
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
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(120), 0.6),
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(315), locManager.getOrientationOrHeading(120))),
                isRed? "P7toP2AASUBRed" : "P7toP2AASUBBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(120)),
                new PathPlannerWaypoint(locManager.P12M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P2AtoP12ASUBRed" : "P2AtoP12ASUBBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerRotationTarget(locManager.getOrientationOrHeading(120), 0.7),
                new PathPlannerWaypoint(locManager.P2A, locManager.getOrientationOrHeading(315), locManager.getOrientationOrHeading(120))),
                isRed? "P12toP2ASUBRed" : "P12toP2ASUBBlue");                

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
                new PathPlannerWaypoint(locManager.P3, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P3toP7MRed" : "P3toP7MBlue");
                
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P7M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P7MtoP7Red" : "P7MtoP7Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P7toP17Red" : "P7toP17Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P12M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P17toP12MRed" : "P17toP12MBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P12M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P12MtoP12Red" : "P12MtoP12Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P12toP17Red" : "P12toP17Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P17toP11MRed" : "P17toP11MBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P11M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P11MtoP11Red" : "P11MtoP11Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed? "P11toP17Red" : "P11toP17Blue");
        
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
                new PathPlannerWaypoint(locManager.P3, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed?"P3toP7Red":"P3toP7Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(270)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(270))),
                isRed?"P7toP6Red":"P7toP6Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(270)),
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(270), locManager.getOrientationOrHeading(270))),
                isRed?"P6toP5Red":"P6toP5Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed?"P6MtoP5MRed":"P6MtoP5MBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P5M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(225)),
                new PathPlannerRotationTarget(180, 0.6),
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed?"P5MtoP5Red":"P5MtoP5Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P3, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed?"P3toP5Red":"P3toP5Blue");


        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P3, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed?"P3toP5Red":"P3toP5Blue");

            addTrajectory(
                trajectoryManager, 
                pathPlanner.buildTrajectory(
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                    new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                    new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))
                    ),
                    isRed?"P7toP12Red":"P7toP12Blue");

            addTrajectory(
                trajectoryManager, 
                pathPlanner.buildTrajectory(
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                    new PathPlannerWaypoint(locManager.P12, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                    new PathPlannerRotationTarget(locManager.getOrientationOrHeading(135), 0.6),
                    new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(135))
                    ),
                    isRed?"P12toP17Red":"P12toP17Blue");

            addTrajectory(
                trajectoryManager, 
                pathPlanner.buildTrajectory(
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                    new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(135)),
                    new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                    new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))
                    ),
                    isRed?"P17toP11Red":"P17toP11Blue"); 

            addTrajectory(
                trajectoryManager, 
                pathPlanner.buildTrajectory(
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                    new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                    new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                    new PathPlannerRotationTarget(locManager.getOrientationOrHeading(135), 0.6),
                    new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(135))
                    ),
                    isRed?"P11toP17Red":"P11toP17Blue"); 

            addTrajectory(
                trajectoryManager, 
                pathPlanner.buildTrajectory(
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                    new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                    new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                    new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                    new PathPlannerWaypoint(locManager.P14, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                    new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),

                    new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                    new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                    new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                    new PathPlannerWaypoint(locManager.P20, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180))
                    ),
                    isRed?"P3toP5Red":"P3toP5Blue");
        
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P2, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(210)), // fix orientation
                new PathPlannerWaypoint(locManager.P5M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5M, locManager.getOrientationOrHeading(255), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P16, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P8M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                "P2toP8");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P11, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed ? "P4toP11Red" : "P4toP11Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed ? "P4toP6Red" : "P4toP6Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P4, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed ? "P4toP5Red" : "P4toP5Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(90), locManager.getOrientationOrHeading(90)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(90), locManager.getOrientationOrHeading(90))),
                isRed ? "P6toP7Red" : "P6toP7Blue");


        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P1, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION))),
            isRed ? "P1toP19Red" : "P1toP19Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION))),
            isRed ? "P19toP8Red" : "P19toP8Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P16, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION))),
            isRed ? "P8toP19Red" : "P8toP19Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION))),
            isRed ? "P19toP9Red" : "P19toP9Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P9M, locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION))),
            isRed ? "P19toP9MRed" : "P19toP9MBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P9M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION))),
            isRed ? "P9MtoP9Red" : "P9MtoP9Blue");
            //TODO
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P16, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P19, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P19_SHOOT_ORIENTATION))),
            isRed ? "P9toP19Red" : "P9toP19Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P10M, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION))),
            isRed ? "P9toP17Red" : "P9toP17Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
            isRed ? "P17toP10Red" : "P17toP10Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION)),
                new PathPlannerWaypoint(locManager.P17, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(TuningConstants.DRIVE_P17_SHOOT_ORIENTATION))),
            isRed ? "P10toP17Red" : "P10toP17Blue");


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