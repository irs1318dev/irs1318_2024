/*
package frc.robot.driver.controltasks;

import frc.lib.driver.IControlTask;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ITimer;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.lib.robotprovider.Point2d;
import frc.robot.AutonLocManager;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ClimberMechanism;
import frc.robot.mechanisms.OffboardVisionManager;


public class TrapMasterTask extends IControlTask
{

    private OffboardVisionManager vision;
    private Itimer timer;
    private double xOffset;
    private double yOffset;
    private double yawOffset;
    private Point2d goalPoint;
    

    public enum RobotState {
        ArmLift,
        Reading,
        Moving,
        Winching,
        ArmMove,
        Outtake
    }
    //ARMLIFT: Move arm so apriltag is readable and hooks are in position.
    //READING: Read Apriltag and determine distance to it
    //MOVING: Move to the desired C Position
    //WINCHING: Winching upwards using ClimberWinchTask
    //ARMMOVE: ArmGraphTask to trap position
    //OUTTAKE: IntakeControlTask to outtake.

    RobotState state = RobotState.Reading;

    @Override
    public void begin()
    {
        
    }



    @Override
    public void update()
    {
        if (this.state == RobotState.ArmLift){
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_TRAP_INTERMEDIATE, TuningConstants.ARM_WRIST_POSITION_TRAP_INTERMEDIATE);


        }

        else if (this.state == RobotState.Reading){
            if (
            this.vision.getAprilTagId() == 1 || //PLACEHOLDERS
            this.vision.getAprilTagId() == 2 ||
            this.vision.getAprilTagId() == 3 ||
            this.vision.getAprilTagId() == 4 ||
            this.vision.getAprilTagId() == 5 ||
            this.vision.getAprilTagId() == 6 
            ) 
            {
            
            }
            //RIGHT HAND RULE (forwards back is x, left right is Y. Front = +. Left = +)
            this.timer = timer;
            this.vision = vision;
            xOffset = this.vision.getAprilTagXOffset();
            yOffset = this.vision.getAprilTagYOffset();
            yawOffset = this.vision.getAprilTagYaw();
            //Heading 0 or calculate using trig
            //Once we know x, y, orientation, and headding then create pathplanner path
        }

        else if (this.state == RobotState.Moving){
            //pathplanner or translation stuff here
            this.state = RobotState.Winching;
        }

        else if (this.state == RobotState.Winching) {
            new ClimberWinchTask(ClimberWinchTask.WinchState.Retracted);
            this.state = RobotState.ArmMove;
        }

        else if (this.state == RobotState.ArmMove){
            new ArmGraphTask(TuningConstants.ARM_SHOULDER_TRAP_OUTTAKE_POS, TuningConstants.ARM_WRIST_TRAP_OUTTAKE_POS);
        }
    }

    @Override
    public void end()
    {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'end'");
    }

    @Override
    public boolean hasCompleted()
    {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'hasCompleted'");
    }

    
    

}

*/