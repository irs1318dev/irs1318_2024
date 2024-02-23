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
                new PathPlannerWaypoint(45.0, 0.0, 0.0, 0.0)),
            "goForwards45in");
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

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P3, locManager.getOrientationOrHeading(225), locManager.getOrientationOrHeading(135)),
                new PathPlannerWaypoint(locManager.P7M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed?"P3ToP7MRed":"P3ToP7MBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P7M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P7, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed?"P7MtoP6MRed":"P7MtoP6MBlue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
                isRed?"P6MtoP6Red":"P6MtoP6Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P6M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P6, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P5M, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(225))),
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
                new PathPlannerWaypoint(locManager.P1, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P16, locManager.getOrientationOrHeading(135), locManager.getOrientationOrHeading(225))),
            isRed ? "P1toP16Red" : "P1toP16Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P16, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(315)),
                new PathPlannerRotationTarget(180, 0.6),
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
            isRed ? "P16toP8Red" : "P16toP8Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P8, locManager.getOrientationOrHeading(45), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P16, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180))),
            isRed ? "P8toP16Red" : "P8toP16Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P16, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
            isRed ? "P16toP9Red" : "P16toP9Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P9, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P13, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P16, locManager.getOrientationOrHeading(0), locManager.getOrientationOrHeading(180))),
            isRed ? "P9toP16Red" : "P9toP16Blue");

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P16, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P18, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180)),
                new PathPlannerWaypoint(locManager.P10, locManager.getOrientationOrHeading(180), locManager.getOrientationOrHeading(180))),
            isRed ? "P16toP10Red" : "P16toP10Blue");


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