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
    private boolean inPIDMode;

    
    //------------------------- Main Arm Initializiation -------------------------
    @Inject
    public LinearActuatorArmMechanism(IDriver driver, IRobotProvider provider, ILogger logger) {
        this.driver = driver;
        this.leftArmLinearActuator = provider.getTalonSRX(10); //change later;
    

        this.leftArmLinearActuator.setSelectedSlot(ArmMechanism.defaultPidSlotId);
        this.leftArmLinearActuator.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.leftArmLinearActuator.setInvertOutput(TuningConstants.ARM_LOWER_LEFT_INVERT_OUTPUT);
        this.leftArmLinearActuator.setInvertSensor(TuningConstants.ARM_LOWER_LEFT_INVERT_SENSOR); 

        this.leftArmLinearActuator.setPosition(0.0);
        
        ITalonSRX rightLowerLAFollower = provider.getTalonSRX(12);
        rightLowerLAFollower.follow(leftArmLinearActuator);     
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
        if (this.driver.getDigital(DigitalOperation.ArmEnableSimpleMode))
        {
            this.inSimpleMode = true;
        }
        else if (this.driver.getDigital(DigitalOperation.ArmUsePID)) {
            this.inPIDMode = true;
        }

        if (this.inSimpleMode)
        {
            this.leftArmLinearActuator.setControlMode(TalonXControlMode.PercentOutput);
            this.leftArmLinearActuator.set(this.driver.getAnalog(AnalogOperation.SimpleMoveLAArm));
        }
        else if (this.inPIDMode) {
            this.leftArmLinearActuator.setControlMode(TalonXControlMode.Position);
            this.leftArmLinearActuator.setPosition(this.driver.getAnalog(AnalogOperation.PIDMoveLAArm))
        }  
    }

    @Override
    public void stop()
    {
       this.leftArmLinearActuator.setPosition(0.0);
    }
   
    

    }