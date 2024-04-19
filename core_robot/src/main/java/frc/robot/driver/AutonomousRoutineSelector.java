 package frc.robot.driver;

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

            // if(routine == AutoRoutine.SixNote)
            // {
                // return Sammamish(locManager, isRed);
            // }

            if(routine == AutoRoutine.Shoot)
            {
                return Shoot(locManager, startPosition, isRed);
            }

            else if(routine == AutoRoutine.Taxi)
            {
                return Taxi();
            }

            else if(routine == AutoRoutine.ShootTaxi)
            {
                return ShootTaxi(locManager, startPosition, isRed);
            }

            else if(routine == AutoRoutine.Sneak)
            {
                return Sneak(locManager, isRed);
            }

            else if(startPosition == StartPosition.WooferFront)
            {
                return SubwooferFrontMultiPiece(locManager, routine, pickupSide, isRed);
            }

            else if(startPosition == StartPosition.Amp)
            {
                return AmpMultiPiece(locManager, routine, isRed);
            }

            else if(startPosition == StartPosition.WooferAmpSide)
            {
                return SubwooferAmpMultiPiece(locManager, routine, isRed);
            }

            else if(startPosition == StartPosition.WooferSourceSide)
            {
                return SubwooferSourceMultiPiece(locManager, routine, isRed);
            }

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

    private static IControlTask Shoot(AutonLocManager locManager, StartPosition position, boolean isRed)
    {
        if(position == StartPosition.WooferFront)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 10.0),
                SequentialTask.Sequence( 
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P4,
                            locManager.getOrientationOrHeading(180),
                            true,
                            true),
                            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                        ),
                    new WaitTask(0.8),
                    new FeedRingTask(true, 3.0)
                )
            );
        }

        else if(position == StartPosition.WooferAmpSide)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 10.0),
                SequentialTask.Sequence( 
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P2A,
                            locManager.getOrientationOrHeading(123.3),
                            true,
                            true),
                            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                        ),
                    new WaitTask(0.8),
                    new FeedRingTask(true, 3.0)
                )
            );
        }

        else if(position == StartPosition.WooferSourceSide)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 10.0),
                SequentialTask.Sequence( 
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P2,
                            locManager.getOrientationOrHeading(236.7),
                            true,
                            true),
                            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                        ),
                    new WaitTask(0.8),
                    new FeedRingTask(true, 3.0)
                )
            );
        }

        else
        {
            return GetFillerRoutine();
        }
    }

    private static IControlTask Taxi()
    {
        return SequentialTask.Sequence(
            new WaitTask(5.0),
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
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new FeedRingTask(true, 3.0),
                    new FollowPathTask(isRed ? "P4toP6STRed" : "P4toP6STBlue", Type.Absolute)
                )
            );
        }

        else if(position == StartPosition.WooferSourceSide)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P2,
                            locManager.getOrientationOrHeading(236.7),
                            true,
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new FeedRingTask(true, 3.0),
                    new FollowPathTask(isRed ? "P2toP19STRed" : "P2toP19STBlue", Type.Absolute)
                )
            );
        }

        else
        {
            return GetFillerRoutine();
        }
    }

    private static IControlTask Sneak(AutonLocManager locManager, boolean isRed)
    {
        return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 5.0),
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P2A,
                            locManager.getOrientationOrHeading(123.3),
                            true,
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.35),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P2Ato12SNKRed" : "P2Ato12SNKBlue", Type.Absolute),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new IntakeControlTask(true, 4.0)
                    ),

                    new DecisionNoteTask(
                        SequentialTask.Sequence(
                                ConcurrentTask.AllTasks(
                                    new FollowPathTask(isRed ? "P12toP17SNKRed" : "P12toP17SNKBlue", Type.Absolute),
                                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P17_SHOT)
                                ),
        
                                new FeedRingTask(true, 0.35),

                                ConcurrentTask.AllTasks(
                                    new FollowPathTask(isRed ? "P17to11SNKRed" : "P17toP11SNKBlue", Type.Absolute),
                                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                                    new IntakeControlTask(true, 2.0)
                                ),

                                ConcurrentTask.AllTasks(
                                    new FollowPathTask(isRed ? "P11toP17SNKRed" : "P11toP17SNKBlue", Type.Absolute),
                                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P17_SHOT)
                                ),

                                new FeedRingTask(true, 0.35),

                                ConcurrentTask.AllTasks(
                                    new FollowPathTask(isRed ? "P17to10SNKRed" : "P17toP10SNKBlue", Type.Absolute),
                                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                                    new IntakeControlTask(true, 3.0)
                                )
                        ),
                        
                        SequentialTask.Sequence(
                            ConcurrentTask.AllTasks(
                                new FollowPathTask(isRed ? "P12toP11SNKRed" : "P12toP11SNKBlue", Type.Absolute),
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                                new IntakeControlTask(true, 2.0)
                            ),
                                
                            ConcurrentTask.AllTasks(
                                new FollowPathTask(isRed ? "P11to17SNKRed" : "P11toP17SNKBlue", Type.Absolute),
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P17_SHOT)
                            ),

                            new FeedRingTask(true, 0.35),

                            ConcurrentTask.AllTasks(
                                new FollowPathTask(isRed ? "P17toP10SNKRed" : "P17toP10SNKBlue", Type.Absolute),
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                                new IntakeControlTask(true, 3.0)
                            )
                        )
                    )
                )
            );
    }
    

    private static IControlTask SubwooferAmpMultiPiece(AutonLocManager locManager, AutoRoutine routine, boolean isRed)
    {
        if (routine == AutoRoutine.TwoNote)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence( 
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P2A,
                            locManager.getOrientationOrHeading(123.3),
                            true,
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.35),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P2AtoP7ASUBRed" : "P2AtoP7ASUBBlue", Type.Absolute),
                        new IntakeControlTask(true, 2.0)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                        new FollowPathTask(isRed ? "P7toP2AASUBRed" : "P7toP2AASUBBlue", Type.Absolute),
                        new IntakeFixerTask(0.15)
                    ),

                    new FeedRingTask(true, 0.25)
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
                            locManager.P2A,
                            locManager.getOrientationOrHeading(123.3),
                            true,
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.35),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P2AtoP7ASUBRed" : "P2AtoP7ASUBBlue", Type.Absolute),
                        new IntakeControlTask(true, 2.0)
                    ),

                    ConcurrentTask.AllTasks(
                        new IntakeFixerTask(0.15),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                        new FollowPathTask(isRed ? "P7toP2AASUBRed" : "P7toP2AASUBBlue", Type.Absolute)
                    ),

                    new FeedRingTask(true, 0.35),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        ConcurrentTask.AnyTasks(
                            new FollowPathTask(isRed ? "P2AtoP12ASUBRed" : "P2AtoP12ASUBBlue", Type.Absolute),
                            SequentialTask.Sequence(
                                new WaitTask(2.0),
                                new IntakeControlTask(true, 10.0)
                            )
                        )
                    )

                    // new DecisionNoteTask(
                        // ConcurrentTask.AllTasks(
                            // new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P17_SHOT),
                            // new FollowPathTask(isRed ? "P12toP17SNKRed" : "P12toP17SNKBlue", Type.Absolute),
                            // new IntakeFixerTask(0.15)
                        // ),
// 
                        // SequentialTask.Sequence(
                            // ConcurrentTask.AllTasks(
                                // new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                                // new FollowPathTask(isRed ? "P12toP11ASUBRed" : "P12toP11ASUBBlue", Type.Absolute)
                            // ),
// 
                            // ConcurrentTask.AllTasks(
                                // new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                                // new FollowPathTask(isRed ? "P11toP17SNKRed" : "P11toP17SNKBlue", Type.Absolute),
                                // new IntakeFixerTask(0.15)
                            // )
                        // )
                    // ),

                    // new FeedRingTask(true, 0.25)
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
                            locManager.P2A,
                            locManager.getOrientationOrHeading(123.3),
                            true,
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.35),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P2AtoP7ASUBRed" : "P2AtoP7ASUBBlue", Type.Absolute),
                        new IntakeControlTask(true, 2.0)
                    ),

                    ConcurrentTask.AllTasks(
                        new IntakeFixerTask(0.1),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                        new FollowPathTask(isRed ? "P7toP2AASUBRed" : "P7toP2AASUBBlue", Type.Absolute)
                    ),

                    new FeedRingTask(true, 0.35),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        ConcurrentTask.AnyTasks(
                            new FollowPathTask(isRed ? "P2AtoP12ASUBRed" : "P2AtoP12ASUBBlue", Type.Absolute),
                            SequentialTask.Sequence(
                                new WaitTask(2.0),
                                new IntakeControlTask(true, 10.0)
                            )
                        )
                    ),

                    new FeedRingTask(true, 0.25),

                    new DecisionNoteTask(
                        SequentialTask.Sequence(
                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                                new FollowPathTask(isRed ? "P12toP17SNKRed" : "P12toP17SNKBlue", Type.Absolute),
                                new IntakeFixerTask(0.15)
                            ),

                            new FeedRingTask(true, 0.7),

                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                                new FollowPathTask(isRed ? "P17to11SNKRed" : "P17to11SNKBlue", Type.Absolute),
                                new IntakeControlTask(true, 3.0)
                            ),

                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                                new FollowPathTask(isRed ? "P11toP17SNKRed" : "P11toP17SNKBlue", Type.Absolute),
                                new IntakeFixerTask(0.15)
                            ),

                            new FeedRingTask(true, 0.7)
                        ),

                        SequentialTask.Sequence(
                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                                new FollowPathTask(isRed ? "P12toP11ASUBRed" : "P12toP11ASUBBlue", Type.Absolute),
                                new IntakeControlTask(true, 2.0)
                            ),

                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                                new FollowPathTask(isRed ? "P11toP17SNKRed" : "P11toP17SNKBlue", Type.Absolute),
                                new IntakeFixerTask(0.15)
                            ),

                            new FeedRingTask(true, 0.35),

                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                                new FollowPathTask(isRed ? "P17to10SNKRed" : "P17to10SNKBlue", Type.Absolute),
                                new IntakeControlTask(true, 3.0)
                            )
                        )
                    )
                )
            );
        }

        else
        {
            return GetFillerRoutine();
        }
    }

    private static IControlTask SubwooferSourceMultiPiece(AutonLocManager locManager, AutoRoutine routine, boolean isRed)
    {
        if(routine == AutoRoutine.TwoNote)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence( 
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P2,
                            locManager.getOrientationOrHeading(236.7),
                            true,
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.35),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P2toP8SSRed" : "P2toP8SSBlue", Type.Absolute),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new IntakeControlTask(true, 4.5)
                    ),

                    new DecisionNoteTask(
                        SequentialTask.Sequence(
                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                                new FollowPathTask(isRed ? "P8toP2AfterSSRed" : "P8toP2AfterSSBlue", Type.Absolute),
                                new IntakeFixerTask(0.15)
                            )
                        ),

                        SequentialTask.Sequence(
                            ConcurrentTask.AllTasks(
                                new FollowPathTask(isRed ? "P8toP9SSRed" : "P8toP9SSBlue", Type.Absolute),
                                new IntakeControlTask(true, 2.0)
                            ),
                            
                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                                new FollowPathTask(isRed ? "P9toP2AfterSSRed" : "P9toP2AfterSSBlue", Type.Absolute),
                                new IntakeFixerTask(0.15)
                            )
                        )
                    ),

                    new FeedRingTask(true, 0.25)
                )
            );
        }

        if(routine == AutoRoutine.ThreePickupNote)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 5.0),
                SequentialTask.Sequence( 
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P2,
                            locManager.getOrientationOrHeading(236.7),
                            true,
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.35),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P2toP8SSRed" : "P2toP8SSBlue", Type.Absolute),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new IntakeControlTask(true, 4.5)
                    ),

                    new DecisionNoteTask(
                        SequentialTask.Sequence(
                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                                new FollowPathTask(isRed ? "P8toP2VisionSSRed" : "P8toP2VisionSSBlue", Type.Absolute),
                                new IntakeFixerTask(0.15)
                            )
                        ),

                        SequentialTask.Sequence(
                            ConcurrentTask.AllTasks(
                                new FollowPathTask(isRed ? "P8toP9SSRed" : "P8toP9SSBlue", Type.Absolute),
                                new IntakeControlTask(true, 2.0)
                            ),
                            
                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                                new FollowPathTask(isRed ? "P9toP2VisionSSRed" : "P9toP2VisionSSBlue", Type.Absolute),
                                new IntakeFixerTask(0.15)
                            )
                        )
                    ),

                    ConcurrentTask.AnyTasks(
                        new VisionShooterTurnAndAimRelativeTask(),
                        SequentialTask.Sequence(
                            new WaitTask(2.5),
                            new FeedRingTask(true, 2.0)
                        )
                    )
                    
                )
            );
        }

        if(routine == AutoRoutine.ThreeNote)
        {
            return ConcurrentTask.AllTasks(
                new ShooterSpinTask(4050, 15.0),
                SequentialTask.Sequence( 
                    ConcurrentTask.AllTasks(
                        new ResetLevelTask(),
                        new PositionStartingTask(
                            locManager.P2,
                            locManager.getOrientationOrHeading(236.7),
                            true,
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.35),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P2toP8SSRed" : "P2toP8SSBlue", Type.Absolute),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new IntakeControlTask(true, 3.5)
                    ),

                    new DecisionNoteTask(
                        SequentialTask.Sequence(
                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P19S_SHOT),
                                new FollowPathTask(isRed ? "P8toP19SSSRed" : "P8toP19SSSBlue", Type.Absolute),
                                new IntakeFixerTask(0.15)
                            ),

                            new FeedRingTask(true, 0.35),

                            ConcurrentTask.AllTasks(
                                new FollowPathTask(isRed ? "P19StoP9SSRed" : "P19StoP9SSBlue", Type.Absolute),
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                                new IntakeControlTask(true, 2.0)
                            ),

                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P19S_SHOT),
                                new FollowPathTask(isRed ? "P9toP19SSSRed" : "P9toP19SSSBlue", Type.Absolute),
                                new IntakeFixerTask(0.15)
                            ),

                            new FeedRingTask(true, 0.25)
                        ),

                        SequentialTask.Sequence(
                            ConcurrentTask.AllTasks(
                                new FollowPathTask(isRed ? "P8toP9SSRed" : "P8toP9SSBlue", Type.Absolute),
                                new IntakeControlTask(true, 2.0)
                            ),
                            
                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P19S_SHOT),
                                new FollowPathTask(isRed ? "P9toP19SSSRed" : "P9toP19SSSBlue", Type.Absolute),
                                new IntakeFixerTask(0.15)
                            ),

                            new FeedRingTask(true, 0.35),

                            ConcurrentTask.AllTasks(
                                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                                new FollowPathTask(isRed ? "P19StoP10SSRed" : "P19StoP10SSBlue", Type.Absolute),
                                new IntakeControlTask(true, 2.0)
                            )
                        )
                    )
                )
            );
        }

        else
        {
            return GetFillerRoutine();
        }
    }

    private static IControlTask SubwooferFrontMultiPiece(AutonLocManager locManager, AutoRoutine routine, PriorityPickupSide pickupSide, boolean isRed)
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
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.35), //shoot

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P4toP6CSRed" : "P4toP6CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 2.0) //go to p6
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                        new FollowPathTask(isRed ? "P6toP4CSRed" : "P6toP4CSBlue", Type.Absolute), //comes back
                        new IntakeFixerTask(0.15)
                    ),

                    new FeedRingTask(true, 0.25) //shoot
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
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.35), //shoot

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P4toP6CSRed" : "P4toP6CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 2.0) //go to p6
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                        new FollowPathTask(isRed ? "P6toP4CSRed" : "P6toP4CSBlue", Type.Absolute), //comes back
                        new IntakeFixerTask(0.15)

                    ),

                    new FeedRingTask(true, 0.35), //shoot

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P4toP7CSRed" : "P4toP7CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 2.5)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                        new FollowPathTask(isRed ? "P7toP4CSRed" : "P7toP4CSBlue", Type.Absolute),
                        new IntakeFixerTask(0.15)
                    ),
                    
                    new FeedRingTask(true, 0.25)
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
                            locManager.P4,
                            locManager.getOrientationOrHeading(180),
                            true,
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, 19.5)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.25), //shoot

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P4toP6CSRed" : "P4toP6CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 1.4) //go to p6
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, 19.5),
                        new FollowPathTask(isRed ? "P6toP4CSRed" : "P6toP4CSBlue", Type.Absolute), //comes back
                        new IntakeFixerTask(0.15)
                    ),

                    new FeedRingTask(true, 0.25), //shoot

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P4toP7CSRed" : "P4toP7CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 2.5)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, 19.5),
                        new FollowPathTask(isRed ? "P7toP4CSRed" : "P7toP4CSBlue", Type.Absolute),
                        new IntakeFixerTask(0.15)
                    ),
                    
                    new FeedRingTask(true, 0.25),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P4toP5CSRed" : "P4toP5CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 2.5)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, 19.5),
                        new FollowPathTask(isRed ? "P5toP4CSRed" : "P5toP4CSBlue", Type.Absolute),
                        new IntakeFixerTask(0.15)
                    ),
                    
                    new FeedRingTask(true, 0.25)
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
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, 19.5)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.25), //shoot

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P4toP6CSRed" : "P4toP6CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 1.4) //go to p6
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, 19.5),
                        new FollowPathTask(isRed ? "P6toP4CSRed" : "P6toP4CSBlue", Type.Absolute), //comes back
                        new IntakeFixerTask(0.15)
                    ),

                    new FeedRingTask(true, 0.25), //shoot

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P4toP5CSRed" : "P4toP5CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 2.5)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, 19.5),
                        new FollowPathTask(isRed ? "P5toP4CSRed" : "P5toP4CSBlue", Type.Absolute),
                        new IntakeFixerTask(0.15)
                    ),
                    
                    new FeedRingTask(true, 0.25)
                )
            );
        }

        else if (routine == AutoRoutine.FiveNote)
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
                            true),
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT)
                    ),

                    new WaitTask(0.7),
                    new FeedRingTask(true, 0.35), //shoot

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P4toP6CSRed" : "P4toP6CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 2.0) //go to p6
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6M_SHOT),
                        new FollowPathTask(isRed ? "P6toP6MCSRed" : "P6toP6MCSBlue", Type.Absolute), //comes back
                        new IntakeFixerTask(0.15)
                    ),

                    new FeedRingTask(true, 0.35), //shoot

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P6MtoP7CSRed" : "P6MtoP7CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 1.25)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6M_SHOT),
                        new FollowPathTask(isRed ? "P7toP6MCSRed" : "P7toP6MCSBlue", Type.Absolute),
                        new IntakeFixerTask(0.15)
                    ),
                    
                    new FeedRingTask(true, 0.35),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P6MtoP5CSRed" : "P6MtoP5CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 1.5)
                    ),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6M_SHOT),
                        new FollowPathTask(isRed ? "P5toP6MCSRed" : "P5toP6MCSBlue", Type.Absolute),
                        new IntakeFixerTask(0.15)
                    ),

                    new FeedRingTask(true, 0.35),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P6MtoP10CSRed" : "P6MtoP10CSBlue", Type.Absolute),
                        new IntakeControlTask(true, 3.5)
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

                    new WaitTask(0.5),
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
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),
                        new FollowPathTask(isRed ? "P7toP6MASRed" : "P7toP6MASBlue", Type.Absolute)
                    ),

                    new WaitTask(0.5),
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

                    new WaitTask(0.5),
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
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),
                        new FollowPathTask(isRed ? "P7toP6MASRed" : "P7toP6MASBlue", Type.Absolute)
                    ),

                    new WaitTask(0.5),
                    new FeedRingTask(true, 0.5),

                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P6MtoP6ASRed" : "P6MtoP6ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 0.5)
                    ),
                    
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),

                    new WaitTask(0.5),
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
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),
                        new FollowPathTask(isRed ? "P7toP6MASRed" : "P7toP6MASBlue", Type.Absolute)
                    ),

                    new FeedRingTask(true, 0.5),

                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),

                    ConcurrentTask.AllTasks(
                        new FollowPathTask(isRed ? "P6MtoP6ASRed" : "P6MtoP6ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 0.5)
                    ),
                    
                    new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_AUTO_P6_SHOT),

                    new FeedRingTask(true, 0.5),

                    ConcurrentTask.AllTasks(
                        new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                        new FollowPathTask(isRed ? "P6toP12ASRed" : "P6toP12ASBlue", Type.Absolute),
                        new IntakeControlTask(true, 3.0)
                    )
                )
            );
        }

        else
        {
            return GetFillerRoutine();
        }
    }
}



//IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS





































































































































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
