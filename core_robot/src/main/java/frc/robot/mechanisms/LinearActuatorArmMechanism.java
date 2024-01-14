package frc.robot.mechanisms;

import frc.robot.*;
import frc.lib.driver.IDriver;
import frc.lib.filters.FloatingAverageCalculator;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.*;
import frc.robot.driver.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ArmMechanism implements IMechanism
{
    //----------------- General variables -----------------

    private static final int defaultPidSlotId = 0;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;
    private final double leftLinearActuatorPosition;
    private final double leftLinearActuatorVelocity;
    //----------------- Main Arm Variables -----------------

    private final ITalonSRX leftArmLinearActuator;
    private final ITalonSRX rightArmLinearActuator;
 
    private boolean inSimpleMode;

    
    //------------------------- Main Arm Initializiation -------------------------
    @Inject
    public LinearActuatorArmMechanism(IDriver driver, IRobotProvider provider, ILogger logger) {
        this.driver = driver;
        this.leftArmLinearActuator = provider.getTalonSRX(ElectronicsConstants.LEFT_LINEAR_ACTUATOR_CAN_ID); 
    
        this.leftArmLinearActuator.setPIDF(
            TuningConstants.LA_ARM_MOTOR_PID_KP, 
            TuningConstants.LA_ARM_MOTOR_PID_KI, 
            TuningConstants.LA_ARM_MOTOR_PID_KD, 
            TuningConstants.LA_ARM_MOTOR_PID_KF,
            LinearActuatorArm.defaultPidSlotId);
            
        this.leftArmLinearActuator.setSelectedSlot(ArmMechanism.defaultPidSlotId);
        this.leftArmLinearActuator.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.leftArmLinearActuator.setInvertOutput(TuningConstants.LINEAR_ACTUATOR_INVER_OUTPUT);
        this.leftArmLinearActuator.setControlMode(TalonXControlMode.Required);
        this.leftArmLinearActuator.setAbsoluteEncoder();
        this.leftArmLinearActuator.setPosition(0.0);
        
        ITalonSRX rightLowerLAFollower = provider.getTalonSRX(ElectronicsConstants.RIGHT_LINEAR_ACTUATOR_CAN_ID);
        rightLowerLAFollower.follow(leftArmLinearActuator);     

        this.leftLinearActuatorPosition = 0.0;
        this.leftLinearActuatorVelocity = 0.0;

        this.inSimpleMode = false; // Add tuning constant
    } 

    @Override
    public void readSensors()
    {
        this.leftLinearActuatorPosition = leftArmLinearActuator.Position;
        this.leftLinearActuatorVelocity = leftArmLinearActuator.Velocity;
        this.logger.logNumber(LoggingKey.LeftLAPosition, this.leftLinearActuatorPosition);
        this.logger.logNumber(LoggingKey.LeftLAVelocity, this.leftLinearActuatorVelocity);
    }

    @Override
    public void update()
    {
        //----------------------------------- Main Arm Control Mode -----------------------------------
        double armPower = this.driver.getAnalog(AnalogOperation.MoveLinearActuatorArm);

        if (this.driver.getDigital(DigitalOperation.ArmEnableSimpleMode))
        {
            this.inSimpleMode = true;
        }
        else if (this.driver.getDigital(DigitalOperation.ArmEnablePID)){
            this.inSimpleMode = false;
        }

        if (this.inSimpleMode)
        {
            this.leftArmLinearActuator.set(TalonXControlMode.PercentOutput, armPower);
        }
        else if (!this.inSimpleMode) {
            this.leftArmLinearActuator.set(TalonXControlMode.Position, armPower);
        }       
    }

    @Override
    public void stop()
    {
       this.leftArmLinearActuator.stop();
    }
}