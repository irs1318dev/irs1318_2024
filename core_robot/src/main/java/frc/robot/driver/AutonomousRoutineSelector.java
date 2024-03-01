package frc.robot.driver;

import java.util.concurrent.TimeUnit;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.driver.IControlTask;
import frc.lib.driver.TrajectoryManager;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.*;
import frc.robot.AutonLocManager;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.SmartDashboardSelectionManager.AutoRoutine;
import frc.robot.driver.SmartDashboardSelectionManager.PriorityPickupSide;
import frc.robot.driver.SmartDashboardSelectionManager.StartPosition;
import frc.robot.driver.controltasks.*;
import frc.robot.driver.controltasks.FollowPathTask.Type;
import frc.robot.driver.controltasks.VisionMoveAndTurnTaskBase.MoveType;
import frc.robot.driver.controltasks.VisionTurningTask.TurnType;

@Singleton
public class AutonomousRoutineSelector
{
    private final ILogger logger;


    private final TrajectoryManager trajectoryManager;
    private final SmartDashboardSelectionManager selectionManager;
    private final IDriverStation driverStation;
    private final AutonLocManager locManager;

    /**
     * Initializes a new AutonomousRoutineSelector
     */
    @Inject
    public AutonomousRoutineSelector(
        LoggingManager logger,
        TrajectoryManager trajectoryManager,
        SmartDashboardSelectionManager selectionManager,
        IRobotProvider provider)
    {
        this.logger = logger;
        this.trajectoryManager = trajectoryManager;
        this.selectionManager = selectionManager;

        this.driverStation = provider.getDriverStation();

        this.locManager = new AutonLocManager(provider);

        RoadRunnerTrajectoryGenerator.generateTrajectories(this.trajectoryManager);
        PathPlannerTrajectoryGenerator.generateTrajectories(this.trajectoryManager, provider.getPathPlanner());
    }

    /**
     * Check what routine we want to use and return it
     * @param mode that is starting
     * @return autonomous routine to execute during autonomous mode
     */
    public IControlTask selectRoutine(RobotMode mode)
    {
        String driverStationMessage = this.driverStation.getGameSpecificMessage();
        this.logger.logString(LoggingKey.AutonomousDSMessage, driverStationMessage);
        if (mode == RobotMode.Test)
        {
            return new WaitTask(0.0);
        }

        if (mode == RobotMode.Autonomous)
        {
            this.locManager.updateAlliance();
            StartPosition startPosition = this.selectionManager.getSelectedStartPosition();
            AutoRoutine routine = this.selectionManager.getSelectedAutoRoutine();
            PriorityPickupSide pickupSide = this.selectionManager.getPickupSide();

            boolean isRed = this.locManager.getIsRed();

            this.logger.logString(LoggingKey.AutonomousSelection, startPosition.toString() + "." + routine.toString());

            if(routine == AutoRoutine.Shoot)
            {
                return Shoot();
            }

            else if(routine == AutoRoutine.Taxi)
            {
                return Taxi();
            }

            else if(routine == AutoRoutine.ShootTaxi)
            {
                return ShootTaxi(locManager, startPosition, isRed);
            }

            else if(startPosition == StartPosition.WooferFront)
            {
                return SubwooferFrontMultiPiece(locManager, routine, isRed);
            }

            else if(startPosition == StartPosition.Amp)
            {
                return AmpMultiPiece(locManager, routine, isRed);
            }

            else if(startPosition == StartPosition.WooferSide)
            {
                return GetFillerRoutine();
            }

            // if(startPosition == StartPosition.Amp && pickupSide == PriorityPickupSide.Close && routine == AutoRoutine.OneNote)
            // {
                // return AmpStartClosePriority(locManager, isRed, 1);
            // }
// 
            // if(startPosition == StartPosition.Amp && pickupSide == PriorityPickupSide.Close && routine == AutoRoutine.TwoNote)
            // {
                // return AmpStartClosePriority(locManager, isRed, 2);
            // }
// 
// 
            // if(startPosition == StartPosition.Amp && pickupSide == PriorityPickupSide.Close && routine == AutoRoutine.ThreeNote)
            // {
                // return AmpStartClosePriority(locManager, isRed, 3);
            // }

            // if(startPosition == StartPosition.WooferFront && pickupSide == PriorityPickupSide.Close && routine == AutoRoutine.TwoNote)
            // {
                // return MidStartNearPriority(locManager, isRed, 2);
            // }
// 
            // if(startPosition == StartPosition.WooferFront && pickupSide == PriorityPickupSide.Close && routine == AutoRoutine.ThreeNote)
            // {
                // return MidStartNearPriority(locManager, isRed, 3);
            // }
// 
            // if(startPosition == StartPosition.Amp && pickupSide == PriorityPickupSide.Center && routine == AutoRoutine.FourNote)
            // {
                // return AmpStartFarPriority(locManager, isRed, 4);
            // }

            else if(startPosition == StartPosition.Source)
            {
                // ALL NEAR SOURCE AUTONS GO HERE
                return GetFillerRoutine();
            }           

            return GetFillerRoutine();
        }

        return GetFillerRoutine();
    }

    /**
     * Gets an autonomous routine that does nothing
     */
    private static IControlTask GetFillerRoutine()
    {
        return new WaitTask(0.0);
    }

    private static IControlTask Shoot()
    {
        return SequentialTask.Sequence(
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
            ConcurrentTask.AnyTasks(
                new ShooterSpinTask(4050, 10.0),
                SequentialTask.Sequence(
                    new WaitTask(3.0),
                    new FeedRingTask(true, 2.0)
                )
            )
        );
    }

    private static IControlTask Taxi()
    {
        return SequentialTask.Sequence(
            new FollowPathTask("goForwards45in", Type.RobotRelativeFromCurrentPose)
        );
    }

    private static IControlTask ShootTaxi(AutonLocManager locManager, StartPosition position, boolean isRed)
    {
        if(position == StartPosition.WooferFront)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P4,
                            locManager.getOrientationOrHeading(180),
                            true,
                            true)
                    ),
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                    new FeedRingTask(true, 3.0),
                    new FollowPathTask(isRed ? "P4toP6Red" : "P4toP6Blue", Type.Absolute)
                )
            );
        }

        else if(position == StartPosition.WooferSide)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P2,
                            locManager.getOrientationOrHeading(225),
                            true,
                            true)
                    ),

                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                    new FeedRingTask(true, 3.0),
                    new FollowPathTask(isRed ? "P2toP19Red" : "P2to19Blue", Type.Absolute)
                )
            );
        }

        else
        {
            return GetFillerRoutine();
        }
    }


    private static IControlTask SubwooferFrontMultiPiece(AutonLocManager locManager, AutoRoutine routine, boolean isRed)
    {
        if (routine == AutoRoutine.TwoNote)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P4,
                            locManager.getOrientationOrHeading(180),
                            true,
                            true)
                    ),

                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                    new FeedRingTask(true, 1.0),


                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P4toP6CSRed" : "P4toP6CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 3.0)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                        new FollowPathTask(isRed ? "P6toP4CSRed" : "P6toP4CSBlue", Type.Absolute)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 1.0)
                )
            );
        }

        else if (routine == AutoRoutine.ThreeNote)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P4,
                            locManager.getOrientationOrHeading(180),
                            true,
                            true)
                    ),

                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                    new FeedRingTask(true, 1.0),


                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P4toP6CSRed" : "P4toP6CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 3.0)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                        new FollowPathTask(isRed ? "P6toP4CSRed" : "P6toP4CSBlue", Type.Absolute)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 1.0),
                
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P4toP7CSRed" : "P4toP7CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 3.0)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                        new FollowPathTask(isRed ? "P7toP4CSRed" : "P7toP4CSBlue", Type.Absolute)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 1.0)
                )
            );
        }

        else if (routine == AutoRoutine.ThreePickupNote)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P4,
                            locManager.getOrientationOrHeading(180),
                            true,
                            true)
                    ),

                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                    new FeedRingTask(true, 1.0),


                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P4toP6CSRed" : "P4toP6CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 2.0)
                    ),

                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),
                    new IntakeControlTask(false, 0.07),
                    new FeedRingTask(true, 1.0),

                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P6toP7CSRed" : "P6toP7CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 3.0)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),
                        new FollowPathTask(isRed ? "P7toP6CSRed" : "P7toP6CSBlue", Type.Absolute),
                        new IntakeControlTask(false, 0.07)
                    ),

                    new FeedRingTask(true, 1.0),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P6toP12CSRed" : "P6toP12CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 4.0)
                    )
                )
            );
        }

        else
        {
            return GetFillerRoutine();
        }
    }

    private static IControlTask AmpMultiPiece(AutonLocManager locManager, AutoRoutine routine, boolean isRed)
    {
        if (routine == AutoRoutine.TwoNote)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P3,
                            locManager.getOrientationOrHeading(180),
                            true,
                            true)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_AUTO_P3_SHOT),
                        new OrientationTask(locManager.getOrientationOrHeading(135), true)
                    ),

                    new FeedRingTask(true, 0.5),

                    ConcurrentTask.AllTasks(
                        new OrientationTask(locManager.getOrientationOrHeading(180), true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
                    ),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P3toP7ASRed" : "P3toP7ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 1.4)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P3_SHOT),
                        new OrientationTask(locManager.getOrientationOrHeading(155), true),
                        new IntakeControlTask(false, 0.07)
                    ),

                    new FeedRingTask(true, 0.5)
                )
            );
        }

        else if (routine == AutoRoutine.ThreeNote)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P3,
                            locManager.getOrientationOrHeading(180),
                            true,
                            true)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_AUTO_P3_SHOT),
                        new OrientationTask(locManager.getOrientationOrHeading(135), true)
                    ),

                    new FeedRingTask(true, 0.5),

                    ConcurrentTask.AllTasks(
                        new OrientationTask(locManager.getOrientationOrHeading(180), true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
                    ),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P3toP7ASRed" : "P3toP7ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 1.4)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P7_SHOT),
                        new OrientationTask(locManager.getOrientationOrHeading(155), true),
                        new IntakeControlTask(false, 0.07)
                    ),

                    new FeedRingTask(true, 0.5),

                    ConcurrentTask.AllTasks(
                        new OrientationTask(locManager.getOrientationOrHeading(180), true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
                    ),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P7toP6ASRed" : "P7toP6ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 1.9)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),
                        new IntakeControlTask(false, 0.07)
                    ),

                    new FeedRingTask(true, 0.5)
                )
            );
        }

        else if (routine == AutoRoutine.ThreePickupNote)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P3,
                            locManager.getOrientationOrHeading(180),
                            true,
                            true)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_AUTO_P3_SHOT),
                        new OrientationTask(locManager.getOrientationOrHeading(135), true)
                    ),

                    new FeedRingTask(true, 0.5),

                    ConcurrentTask.AllTasks(
                        new OrientationTask(locManager.getOrientationOrHeading(180), true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
                    ),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P3toP7ASRed" : "P3toP7ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 1.4)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P7_SHOT),
                        new OrientationTask(locManager.getOrientationOrHeading(155), true),
                        new IntakeControlTask(false, 0.07)
                    ),

                    new FeedRingTask(true, 0.5),

                    ConcurrentTask.AllTasks(
                        new OrientationTask(locManager.getOrientationOrHeading(180), true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
                    ),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P7toP6ASRed" : "P7toP6ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 1.9)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),
                        new IntakeControlTask(false, 0.07)
                    ),

                    new FeedRingTask(true, 0.5),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P6toP12ASRed" : "P6toP12ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 3.0)
                    )
                )
            );
        }

        else if (routine == AutoRoutine.FourNote)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P3,
                            locManager.getOrientationOrHeading(180),
                            true,
                            true)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_AUTO_P3_SHOT),
                        new OrientationTask(locManager.getOrientationOrHeading(135), true)
                    ),

                    new FeedRingTask(true, 0.5),

                    ConcurrentTask.AllTasks(
                        new OrientationTask(locManager.getOrientationOrHeading(180), true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
                    ),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P3toP7ASRed" : "P3toP7ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 1.4)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P7_SHOT),
                        new OrientationTask(locManager.getOrientationOrHeading(155), true),
                        new IntakeControlTask(false, 0.07)
                    ),

                    new FeedRingTask(true, 0.5),

                    ConcurrentTask.AllTasks(
                        new OrientationTask(locManager.getOrientationOrHeading(180), true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
                    ),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P7toP6ASRed" : "P7toP6ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 1.9)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),
                        new IntakeControlTask(false, 0.07)
                    ),

                    new FeedRingTask(true, 0.5),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P6toP12ASRed" : "P6toP12ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 3.0)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),
                        new FollowPathTask(isRed ? "P12toP6ASRed" : "P12toP6ASBlue", Type.Absolute),
                        new IntakeControlTask(false, 0.07)
                    ),

                    new FeedRingTask(true, 0.5)
                )
            );
        }

        else
        {
            return GetFillerRoutine();
        }
    }

    // private static IControlTask AmpStartMiddlePriority(AutonLocManager locManager, boolean isRed, int numberNotes)
    // {
    //     if(numberNotes == 1)
    //     {
    //         return SequentialTask.Sequence(
    //             ConcurrentTask.AllTasks(
    //                 new ResetLevelTask(),
    //                 new PositionStartingTask(
    //                     locManager.P3,
    //                     locManager.getOrientationOrHeading(0),
    //                     true,
    //                     true)
    //         ),

    //         new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
    //         new FollowPathTask(isRed ? "P3RotateToShootRed" : "P3RotateToShootBlue", Type.Absolute),
    //         new FeedRingTask(true),
    //         new ShooterSpinTask(300, 1),
    //         new FollowPathTask(isRed ? "P3ToP7MRed" : "P3toP7MBlue", Type.Absolute)

    //         );

    //     }

    //     else if(numberNotes == 2)
    //     {
    //         return SequentialTask.Sequence(
    //             ConcurrentTask.AllTasks(
    //                 new ResetLevelTask(),
    //                 new PositionStartingTask(
    //                     locManager.P3,
    //                     locManager.getOrientationOrHeading(0),
    //                     true,
    //                     true)
    //         ),

    //         new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
    //         new FollowPathTask(isRed ? "P3RotateToShootRed" : "P3RotateToShootBlue", Type.Absolute),
    //         new FeedRingTask(true),
    //         new ShooterSpinTask(300, 1),
    //         new FollowPathTask(isRed ? "P3ToP7MRed" : "P3toP7MBlue", Type.Absolute),

    //         ConcurrentTask.AllTasks(
    //         new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
    //         new FollowPathTask(isRed ? "P7MtoP6MRed" : "P7MtoP6MBlue", Type.Absolute),
    //         new IntakeControlTask(true)
    //         ));
    //     }
    // }

    

    private static IControlTask WooferCenterStartOnePlusOne(AutonLocManager locManager, boolean isRed, int numberNotes)
    {
        //unfinished
        if(numberNotes == 1)
        {
            return null;
        }
        else 
        {
            return SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        locManager.P1,
                        locManager.getOrientationOrHeading(180),
                        true,
                        true)
            ),

            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)            
            );
        }
    }

    private static IControlTask SourceStartFarPriority(AutonLocManager locManager, boolean isRed, int numberNotes)
    {
        if(numberNotes == 3)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(3500, 15.0),
                SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        locManager.P1,
                        locManager.getOrientationOrHeading(180),
                        true,
                        true)
            ),
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
            new FollowPathTask(isRed ? "P1toP19Red" : "P1toP19Blue", Type.Absolute),
            new FeedRingTask(true, 0.5),
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
            ConcurrentTask.AllTasks(
                new IntakeControlTask(true, 2.0),
                new FollowPathTask(isRed ? "P19toP8Red" : "P19toP8Blue", Type.Absolute)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "P8toP19Red" : "P8toP19Blue", Type.Absolute),
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
            ),
            new FeedRingTask(true, 0.5),
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "P19toP9Red" : "P19toP9Blue", Type.Absolute),
                new IntakeControlTask(true, 2)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "P9toP17Red" : "P9toP17Blue", Type.Absolute),
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
            ),
            new FeedRingTask(true, 0.5),
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "P17toP10Red" : "P17toP10Blue", Type.Absolute),
                new IntakeControlTask(true, 2)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "P10toP17Red" : "P10toP17Blue", Type.Absolute),
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
            ),
            new FeedRingTask(true, 0.5),
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
            ));
        }
        else
        {
            return GetFillerRoutine();
        }
    }

    private static IControlTask MidStartNearPriority(AutonLocManager locManager, boolean isRed, int numberNotes)
    {
        if(numberNotes == 1)
        {
            return null;
        }
        else if (numberNotes == 2)
        {
            return null;
        }
        else
        {
            return 
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new WaitTask(0.2),
                    new ShooterSpinTask(3500, 15.0)
                ),
                SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        locManager.P4, 
                        locManager.getOrientationOrHeading(180.0),
                        true,
                        true)
            ),

            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
            new FeedRingTask(true, 0.5),
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
            ConcurrentTask.AllTasks(
                new IntakeControlTask(true, 1.5),
                new FollowPathTask(isRed ? "P4toP6Red" : "P4toP6RBlue", Type.Absolute)
            ),
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),
            new IntakeControlTask(false, 0.1),
            new FeedRingTask(true, 0.5),
            
            ConcurrentTask.AllTasks(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                new OrientationTask(locManager.getOrientationOrHeading(90), true)
            ),

            ConcurrentTask.AllTasks(
                new IntakeControlTask(true, 2.0),
                new FollowPathTask(isRed ? "P6toP7Red" : "P6toP7Blue", Type.Absolute)
            ),

            ConcurrentTask.AllTasks(
                new OrientationTask(locManager.getOrientationOrHeading(155), true),
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P3_SHOT),
                new IntakeControlTask(false, 0.1)
            ),
            new FeedRingTask(true, 0.5)
            )
            );
        }
    }



    private static IControlTask AmpStartClosePriority(AutonLocManager locManager, boolean isRed, int numberNotes)
    {

        //if number of notes is 1, grab rightmost note and shoot at P6M
        if(numberNotes == 1)
        {
            return SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        locManager.P3,
                        locManager.getOrientationOrHeading(180),
                        true,
                        true)
            ),

            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_AUTO_P3_SHOT),
            new OrientationTask(locManager.getOrientationOrHeading(135), true),
            ConcurrentTask.AllTasks(
                new ShooterSpinTask(3000, 3),
                new FeedRingTask(true, 3.0)
            ));
        }


        //if number of notes is 2, grab right most note, shoot at P6M, grab center note, shoot in place.
        else if(numberNotes == 2)
        {
            return SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        locManager.P3,
                        locManager.getOrientationOrHeading(180),
                        true,
                        true)
            ),

            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
            new FollowPathTask(isRed ? "P3RotateToShootRed" : "P3RotateToShootBlue", Type.Absolute),
            new FeedRingTask(true),
            new ShooterSpinTask(300, 1),
            new FollowPathTask(isRed ? "P3ToP7MRed" : "P3toP7MBlue", Type.Absolute),

            ConcurrentTask.AllTasks(
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
            new FollowPathTask(isRed ? "P7MtoP6MRed" : "P7MtoP6MBlue", Type.Absolute),
            new IntakeControlTask(true)
            ),

            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
            new FeedRingTask(true),
            new ShooterSpinTask(300, 1),

            ConcurrentTask.AllTasks(
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
            new FollowPathTask(isRed ? "P6Mto6Red" : "P6MtoP6Blue", Type.Absolute),
            new IntakeControlTask(true)
            ),

            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
            new FeedRingTask(true),
            new ShooterSpinTask(300, 1));
        }

        //if number of notes is 3, grab grab right most note, shoot at P6M, grab center note, shoot in place,
        //grab left note
        
        else
        {
            return 
            ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15),
            SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        locManager.P3.x,
                        locManager.P3.y,
                        locManager.getOrientationOrHeading(180),
                        true,
                        true)
            ),

            ConcurrentTask.AllTasks(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_AUTO_P3_SHOT),
                new OrientationTask(locManager.getOrientationOrHeading(135), true)
            ),

            new FeedRingTask(true, 0.5),

            ConcurrentTask.AllTasks(
                new OrientationTask(locManager.getOrientationOrHeading(180), true),
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "P3toP7Red" : "P3toP7Blue", Type.Absolute),
                new IntakeControlTask(true, 1.4)
            ),

            ConcurrentTask.AllTasks(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P3_SHOT),
                new OrientationTask(locManager.getOrientationOrHeading(155), true),
                new IntakeControlTask(false, 0.07)
            ),
            
            new FeedRingTask(true, 0.5),

            ConcurrentTask.AllTasks(
                new OrientationTask(locManager.getOrientationOrHeading(270), true),
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "P7toP6Red" : "P7toP6Blue", Type.Absolute),
                new IntakeControlTask(true, 1.4)
            ),

            ConcurrentTask.AllTasks(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P5_SHOT),
                new OrientationTask(locManager.getOrientationOrHeading(180), true),
                new IntakeControlTask(false, 0.07)
            ),
            
            new FeedRingTask(true, 0.5),

            ConcurrentTask.AllTasks(
                new OrientationTask(locManager.getOrientationOrHeading(270), true),
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "P6toP5Red" : "P6toP5Blue", Type.Absolute),
                new IntakeControlTask(true, 1.4)
            ),

            ConcurrentTask.AllTasks(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P5_SHOT),
                new OrientationTask(locManager.getOrientationOrHeading(205), true),
                new IntakeControlTask(false, 0.07)
            ),
            
            new FeedRingTask(true, 0.5)

            ));
        }
    }

    private static IControlTask AmpStartFarPriority(AutonLocManager locManager, boolean isRed, int numberNotes)
    {
        //if number of notes is 1, grab rightmost note and shoot at P6M
        if(numberNotes == 1)
        {
            return SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        locManager.P3,
                        locManager.getOrientationOrHeading(180),
                        true,
                        true)
            )

            );
        }


        //if number of notes is 2, grab right most note, shoot at P6M, grab center note, shoot in place.
        else if(numberNotes == 2)
        {
            return SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        locManager.P3,
                        locManager.getOrientationOrHeading(180),
                        true,
                        true)
            )

            );
        }

        //if number of notes is 3, grab grab right most note, shoot at P6M, grab center note, shoot in place,
        //grab left note
        
        else
        {
            return 
            ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15),
            SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        locManager.P3,
                        locManager.getOrientationOrHeading(180),
                        true,
                        true)
            ),

            ConcurrentTask.AllTasks(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_AUTO_P3_SHOT),
                new OrientationTask(locManager.getOrientationOrHeading(135), true)
            ),

            new FeedRingTask(true, 0.5),

            ConcurrentTask.AllTasks(
                new OrientationTask(locManager.getOrientationOrHeading(180), true),
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "P3toP7Red" : "P3toP7Blue", Type.Absolute),
                new IntakeControlTask(true, 1.4)
            ),

            ConcurrentTask.AllTasks(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P3_SHOT),
                new OrientationTask(locManager.getOrientationOrHeading(155), true),
                new IntakeControlTask(false, 0.07)
            ),
            
            new FeedRingTask(true, 0.5),

            ConcurrentTask.AllTasks(
                new OrientationTask(locManager.getOrientationOrHeading(180), true),
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP)
            ),

            ConcurrentTask.AllTasks(
                new FollowPathTask(isRed ? "P7toP12Red" : "P7toP12Blue", Type.Absolute),
                new IntakeControlTask(true, 3.4)
            ),

            ConcurrentTask.AllTasks(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P5_SHOT),
                new FollowPathTask(isRed ? "P12toP17Red" : "P12toP17Blue", Type.Absolute),
                new IntakeControlTask(false, 0.07)
            ),
            
            new FeedRingTask(true, 0.5),

            ConcurrentTask.AllTasks(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                new FollowPathTask(isRed ? "P17toP11Red" : "P17toP11Blue", Type.Absolute),
                new IntakeControlTask(true, 3.4)
            ),

            ConcurrentTask.AllTasks(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P5_SHOT),
                new FollowPathTask(isRed ? "P11toP17Red" : "P11toP17Blue", Type.Absolute),
                new IntakeControlTask(false, 0.07)
            ),
            
            new FeedRingTask(true, 0.5)

            ));
        }
    }
}









































































































































/*
                                      .
                                    .;+;+
                                    .+;;'   `,+'.
                                    ;';;+:..`` :+'+
                                    ,'+`    .+;;;;;+
                                     ;,,, .+;;;;;'+++;
                                     ;' `+;;;;;#+'+'+''#:.
                                     '`+';;;'+;+;+++'''+'.
                                     #';;;;#';+'+'''+''+'
                                     ;;;;#;,+;;+;;;'''''':
                                     ';'++'.`+;;'';;''+'',
                                     :#'#+'``.'+++'#++'':`
                                      `';++##```##+.''.##
                                      +++#   #`#  `++++
                                      +'#+ # :#: # ##'+
                                      `#+#   +`+   #'#`
                                       :,.+,+,`:+,+..,
                                       `,:```,`,`.`;,
                                        :+.;``.``;.#;
                                        .'``'+'+'``'.
                                         ,````````..
                                          :```````:
                                          +``.:,``'
                                          :```````:
                                           +`````+
                                            ';+##
                                            '```'
                                           `'```'`
                                         .+''''''''
                                        +;;;;;;;;''#
                                       :       `   `:
                                      `,            '
                                      +              '
                                     ,;';,``.``.,,,:;#
                                     +;;;;;;;;;;;;;;;'
                                    ,';;;;;;;;;;;;;;;',
                                    +:;;;;;;';;;;;;;;;+
                                   `.   .:,;+;;:::;.``,
                                   :`       #,       `.`
                                   +       # ;        .;
                                  .;;,`    ,         `,+
                                  +;;;;;;''';;;;;;;';;';
                                  +;;;;;;;';;;;;;;;;;'';;
                                 `';;;;;;';;;;;;;;;;;';;+
                                 + `:;;;;+;;;;;;;;';'''::
                                 '     `:  ```````    ,  ,
                                :       '             ;  +
                                '`     ..             ,  ,
                               ,;;;;;..+,`        ```.':;',
                               +;;;;;;'+;;;;;;;;;;;;;;+;;;+
                               ';;;;;;++;;;;;;;;;;;;;;';;;+
                              `.:';;;;;#;;;;;;;;;;;;;;';;;;`
                              ;    `,; ',:;;';;';;;;;:;``  +
                              +      ; ;              ;    `
                              ;      : +              '    `;
                              ';:`` `` '              :`,:;;+
                             `';;;;'+  +,..```````..:;#;;;;;;.
                             `;;;;;;+  +;;;;;;;;;;;;;':';;;;;#
                             .;;;;;;+  ';;;;;;;;;;;;;;,';;;;` .
                             : `.;;'+  +;;;;;;;;;;;;;','.`    +
                             '      ;  +.,,;:;:;;;,..`: ,     ``
                             +      ,  '              : ;   .;'+
                             +.`   ``  +              ;  ;:;;;;':
                             ';;;';;`  +             .'  ;;;;;;;+
                             ';;;;;'   :+++#++##+#+''',   +;;;;.`.
                             +;;;;;'   +;;::;;;+:+;;'',   ,;;.   +
                            ``:;;;;+   +;;:;;;:+;+;;++;    +     .`
                             `   ``'   +;;;;;;;+;+;;'+;     ,   ;#,
                            .      ;   ';;;;;;;;;;;;++'     + .+``.;
                            ``     ;   ';;;;;;+;';;;'+'      #`````:,
                             +++;,:.   ':;''++;:';:;'';      +``````,`
                             ,```,+    +;;';:;;+;;;;'';      +``````,+
                            .``````:   ;:;;++';;;;;;';,      ,``:#``+`.
                            ,``````'   `';;;;:;;;;;;+;`     '+``+:'`..'
                            ,``````'    +;;;;;;;;;;;''     ;:'``#;;.`++
                            ```````;    `;:;;;;;;;;;;#     ':'``++:+`+;
                            ```'`.`;     +;;;;;;;;;;;+    :::#``' +#`';
                            ,``'`:`#     `';;;;;;;;;;+    +:'.`,. ++`;;
                            +`.``+`'     :#;;;;;;;;;;;`   +:# ,`  +;`.'
                           ,.`+`.:.      ##;;;;;;;;;;;'   ,'`     ;:+#
                           '`;.`+`#      ##+;;;;;;;;;;+          ,::;
                           ,+,`:``,     :###;;;;;;;;;:'          +:;`
                            '`,,`+      ';##';;;;;;;;;;.         +:#
                             '+.+       +;;##;;;;;;;;;;'         ;:;
                               `       :;;;+#;;;;;;;;;;+        ;::`
                                       +;;;;#+;;;;;;;;;;        +:'
                                       ';;;;+#;;;;;;;;;;.       ;:'
                                      ,;;;;;;#;;;;;;;;;;+      +::.
                                      +;;;;;;'';;;;;;;;;'      +:+
                                     `;;;;;;;;#;;;;;;;;;;`    `;:+
                                     ,;;;;;;;;+;;;;;;;;;;+    ':;,
                                     +;;;;;;;;;+;;;;;;;;;'    +:+
                                    .;;;;;;;;;+,;;;;;;;;;;`   ;;+
                                    ';;;;;;;;;, ';;;;;;:;;,  +;:,
                                    ';;;;;;;;'  +;;;;;;;;;'  +:+
                                   ;;;;;;;;;;+  ,;;;;;;;;;+  ;:'
                                   +;;;;;;;;;    ';;;;;;;;;`;:;`
                                   ;;;;;;;;;+    +;;;;;;;;;+#:+
                                  ';;;;;;;;;:    ;;;;;;;;;;';:'
                                 `';;;;;;;:'      ';;;;;;;;;;:.
                                 .;;;;;;;;;+      +;;;;;;;;;'+
                                 +;;;;;;;;;       ';;;;;;;;;#+
                                `;;;;;;;;;+       `;;;;;;;;;;`
                                +;;;;;;;;;.        +;;;;;;;;;`
                                ';;;;;;;:'         ;;;;;;;;;;;
                               :;;;;;;;;;:         `;;;;;;;;;+
                               +;;;;;;;;;           ';;;;;;;;;`
                               ;;;;;;;;;+           ';;;;;;;;;:
                              ';;;;;;;;;;           ,;;;;;;;;;+
                              ':;;;;;;;'             +;;;;;;;;;
                             .;:;;;;;;;'             +;;;;;;;;;:
                             +;;;;;;;;;`             .;;;;;;;;;+
                            `;;;;;;;;;+               ;:;;;;;;;;`
                            ;;;;;;;;;;.               +;;;;;;;::.
                            ';;;;;;;;'`               :;;;;;;;;:+
                           :;;;;;;;;:'                ';;;;;;;;;'
                           ';;;;;;;;'`                +#;;;;;;;;;`
                          `;;;;;;;;;+                 '';;;;;;;;;+
                          +;;;;;;;;;.                '::;;;;;;;;;+
                          ;;;;;;;;;+                 #:'';;;;;;;;;`
                         .#;;;;;;;;'                `;:+;;;;;;;;;;;
                         ':'';;;;;;                 '::.,;;;;;;;;;+
                        +::::+';;;+                 ':'  +:;;;;;;;;`
                       `;;;::::;#+:                `;:+  +;;;;;;;:;;      '#+,
                       +#::::::::;'`               +:;,  `;;;;:;;'#';;;;;::;:'`
                      ';:''::::::::#`              +:'    ';:;;+'::;;:;::::::''
                      #+::;+':::::::'.            .:;+    '''+;::;:;:::;:::;':'
                    `';+';;:;'';:::::':    '      +::.     +:::::::::::::;#;:#
                    :+;#'.''##;#;:;;:::'+  #     `+;'      ;:;::::::::;'+;:'+
                   '#;+". ` `+:;+:;::;::+'#+     +:;#     ';:::;:+#+';:::+.
                   ';#''      ,+::+#';::;+'#+    ';::      #:;;'+';'''++:`
                                '':::;'''#+     ,:;;`      #';:;;:+
                                 `:'++;;':       :++       .;;:;;#,
                                       `                    '':``


*/
