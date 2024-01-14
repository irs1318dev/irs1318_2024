package frc.robot.mechanisms;

import frc.lib.mechanisms.*;
import frc.lib.robotprovider.ITalonFX;
import frc.lib.robotprovider.ITalonSRX;
import frc.lib.robotprovider.TalonFXControlMode;
import frc.robot.ElectronicsConstants;
import frc.robot.driver.DigitalOperation;
import src.main.java.frc.lib.driver.IDriver;
import src.main.java.frc.lib.mechanisms.IMechanism;
import src.main.java.frc.lib.robotprovider.ILogger;
import src.main.java.frc.lib.robotprovider.IRobotProvider;
import src.main.java.frc.lib.robotprovider.ITimer;
import src.main.java.frc.robot.LoggingKey;
import src.main.java.frc.robot.TuningConstants;

//Inject Functions
import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ChainArmMechanism implements IMechanism{

    private static final int defaultPidSlotId = 0;

    private final IDriver driver;
    private final ILogger logger;
    private final double leftMotorPosition;
    private final double leftMotorVelocity;
    private final ITalonSRX rightMotor;
    private final ITalonSRX leftMotor;

    @Inject
    public ChainArmMechanism(IRobotProvider provider, IDriver driver)
    {
        this.driver = driver;
        this.rightMotor = provider.getTalonSRX(ElectronicsConstants.LEFT_CHAIN_MOTOR_CAN_ID);
        this.leftMotor = provider.getTalonSRX(ElectronicsConstants.RIGHT_CHAIN_MOTOR_CAN_ID);
        this.leftMotor.setSelectedSlot(defaultPidSlotId);
        this.leftMotor.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.leftMotor.setInvertOutput(TuningConstants.CHAIN_ARM_ACTUATOR_INVER_OUTPUT); 
        this.leftMotor.setPosition(0.0);
        this.leftMotor.setControlMode(TalonXControlMode.Required);
        this.leftMotor.setPIDF(
            TuningConstants.CHAIN_ARM_MOTOR_PID_KP, 
            TuningConstants.CHAIN_ARM_MOTOR_PID_KI, 
            TuningConstants.CHAIN_ARM_MOTOR_PID_KD, 
            TuningConstants.CHAIN_ARM_MOTOR_PID_KF, 
            ChainArmMechanism.defaultPidSlotId);
        this.rightMotor.follow(this.leftMotor);
    }
    
    @Override
    public void readSensors()
    {
        this.leftMotorPosition = leftMotor.Position;
        this.leftMotorVelocity = leftMotor.Velocity;
        this.logger.logNumber(LoggingKey.LeftMotorPosition, this.leftMotorPosition);
        this.logger.logNumber(LoggingKey.LeftMotorVelocity, this.leftMotorVelocity);
    }

    @Override
    public void update()
    {
        double armPower = this.leftMotor.setPosition(this.driver.getAnalog(AnalogOperation.MoveChainArm));

        if(DigitalOperation.ArmUsePID)
        {
            this.leftMotor.set(TalonXControlMode.Position, armPower);
        }
        
        else if(DigitalOperation.ArmUsePercentOutput)
        {
            this.leftMotor.set(TalonXControlMode.PercentOutput, armPower);
        }
        
    }

    @Override
    public void stop()
    {
        this.leftMotor.stop();
    }
}
