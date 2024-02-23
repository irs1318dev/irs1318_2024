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

            if(routine == AutoRoutine.Taxi)
            {
                return Taxi();
            }

            if(routine == AutoRoutine.ShootTaxi)
            {
                return ShootTaxi();
            }

            if(startPosition == StartPosition.Amp && pickupSide == PriorityPickupSide.Close && routine == AutoRoutine.OneNote)
            {
                return AmpStartClosePriority(locManager, isRed, 1);
            }

            if(startPosition == StartPosition.Amp && pickupSide == PriorityPickupSide.Close && routine == AutoRoutine.TwoNote)
            {
                return AmpStartClosePriority(locManager, isRed, 2);
            }


            if(startPosition == StartPosition.Amp && pickupSide == PriorityPickupSide.Close && routine == AutoRoutine.ThreeNote)
            {
                return AmpStartClosePriority(locManager, isRed, 3);
            }

            else if(startPosition == StartPosition.Source)
            {
                // ALL NEAR SOURCE AUTONS GO HERE
                return GetFillerRoutine();
            }

            else if(startPosition == StartPosition.WooferFront)
            {
                // ALL NEAR SUBWOOFER FRONT AUTONS GO HERE
                return GetFillerRoutine();
            }

            else if(startPosition == StartPosition.WooferSide)
            {
                // ALL NEAR SUBWOOFER SIDE AUTONS GO HERE
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
                new ShooterSpinTask(2500),
                new WaitTask(2.0)
            ),
            ConcurrentTask.AnyTasks(
                new ShooterSpinTask(2500),
                new FeedRingTask(true, 3.0)
            )
        );
    }

    private static IControlTask Taxi()
    {
        return SequentialTask.Sequence(
            new FollowPathTask("goForwards45in", Type.RobotRelativeFromCurrentPose)
        );
    }

    private static IControlTask ShootTaxi()
    {
        return SequentialTask.Sequence(
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
            ConcurrentTask.AnyTasks(
                new ShooterSpinTask(2500),
                new WaitTask(2.0)
            ),
            ConcurrentTask.AnyTasks(
                new ShooterSpinTask(2500),
                new FeedRingTask(true, 3.0)
            ),
            new FollowPathTask("goForwards45in", Type.RobotRelativeFromCurrentPose)
        );
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

            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_AUTO_P3_SHOOT),
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
            return SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ResetLevelTask(),
                    new PositionStartingTask(
                        locManager.P3,
                        locManager.getOrientationOrHeading(180),
                        true,
                        true)
            ),

            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_AUTO_P3_SHOOT),
            new WaitTask(0.5),
            new OrientationTask(locManager.getOrientationOrHeading(135), true),
            ConcurrentTask.AnyTasks(
                new ShooterSpinTask(3500, 3),
                new FeedRingTask(true, 3.0)
            ),
            new FollowPathTask(isRed ? "P3ToP7MRed" : "P3toP7MBlue", Type.Absolute),

            ConcurrentTask.AllTasks(
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
                new FollowPathTask(isRed ? "P7MtoP6MRed" : "P7MtoP6MBlue", Type.Absolute),
                new IntakeControlTask(true, 3.0)
            ),

            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
            ConcurrentTask.AllTasks(
                new ShooterSpinTask(3000, 3),
                new FeedRingTask(true, 3.0)
            ),

            ConcurrentTask.AllTasks(
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
            new FollowPathTask(isRed ? "P6MtoP5MRed" : "P6MtoP5MBlue", Type.Absolute),
            new IntakeControlTask(true, 3.0)
            ),

            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
            ConcurrentTask.AllTasks(
                new ShooterSpinTask(3000, 3),
                new FeedRingTask(true, 3.0)
            ),

            ConcurrentTask.AllTasks(
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
            new FollowPathTask(isRed ? "P5MtoP5Red" : "P5MtoP5Blue", Type.Absolute),
            new IntakeControlTask(true, 3.0)
            ),
            
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
            new OrientationTask(locManager.getOrientationOrHeading(225), true),
            ConcurrentTask.AllTasks(
                new ShooterSpinTask(3000, 3),
                new FeedRingTask(true, 3.0)
            )
            );
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
