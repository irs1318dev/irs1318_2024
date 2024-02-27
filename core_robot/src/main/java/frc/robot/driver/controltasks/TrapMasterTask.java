package frc.robot.driver.controltasks;

import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ClimberMechanism;

public class TrapMasterTask extends DecisionSequentialTask
{

    public enum RobotState {
        Reading,
        Moving,
        Winching,
        ArmMove,
        Outtake
    }

    

}