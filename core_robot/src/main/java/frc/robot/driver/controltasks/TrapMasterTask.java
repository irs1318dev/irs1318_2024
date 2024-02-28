package frc.robot.driver.controltasks;

import frc.lib.driver.IControlTask;
import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ClimberMechanism;
import frc.robot.mechanisms.OffboardVisionManager;

public class TrapMasterTask extends ControlTaskBase
{

    private OffboardVisionManager vision;
    private Itimer timer;
    private double xOffset;
    private double yOffset;

    public enum RobotState {
        Reading,
        Moving,
        Winching,
        ArmMove,
        Outtake
    }

    //READING: Read Apriltag and determine distance to it
    //MOVING: Move to the desired C Position
    //WINCHING: Winching upwards using ClimberWinchTask
    //ARMMOVE: ArmGraphTask to trap position
    //OUTTAKE: IntakeControlTask to outtake.

    RobotState state = RobotState.Reading;

    @Override
    public void begin()
    {
        this.timer = timer;
        this.vision = vision;
        xOffset = this.vision.getAprilTagXOffset();
        yOffset = this.vision.getAprilTagYOffset();
    }

    @Override
    public void update()
    {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
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