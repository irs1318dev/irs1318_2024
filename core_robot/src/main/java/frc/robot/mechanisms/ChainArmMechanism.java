package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ITalonFX;
import frc.lib.robotprovider.ITalonSRX;
import frc.lib.robotprovider.MotorNeutralMode;
import frc.lib.robotprovider.TalonSRXControlMode;
import frc.lib.robotprovider.TalonSRXFeedbackDevice;
import frc.robot.ElectronicsConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
//Inject Functions
import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ChainArmMechanism implements IMechanism{

    private static final int defaultPidSlotId = 0;

    private final IDriver driver;
    private final ILogger logger;
    private double leftMotorPosition;
    private double leftMotorVelocity;
    private final ITalonSRX leftMotor;
    private final double gearRatio;
    private final double minROM;
    private final double maxROM;

    @Inject
    public ChainArmMechanism(IRobotProvider provider, IDriver driver, ILogger logger)
    {
        this.driver = driver;
        this.logger = logger;

        this.leftMotor = provider.getTalonSRX(ElectronicsConstants.LEFT_CHAIN_MOTOR_CAN_ID);

        this.leftMotor.setPIDF(
            TuningConstants.CHAIN_ARM_MOTOR_PID_KP, 
            TuningConstants.CHAIN_ARM_MOTOR_PID_KI, 
            TuningConstants.CHAIN_ARM_MOTOR_PID_KD, 
            TuningConstants.CHAIN_ARM_MOTOR_PID_KF, 
            ChainArmMechanism.defaultPidSlotId);
<<<<<<< HEAD

        this.leftMotor.setSelectedSlot(defaultPidSlotId);
        this.leftMotor.setSensorType(TalonSRXFeedbackDevice.None);

        this.leftMotor.setPosition(0.0);
        this.leftMotor.setMotorOutputSettings(TuningConstants.CHAIN_ARM_MOTOR_INVER_OUTPUT, MotorNeutralMode.Brake);
        this.leftMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        

        ITalonSRX rightMotor = provider.getTalonSRX(ElectronicsConstants.RIGHT_CHAIN_MOTOR_CAN_ID);
        rightMotor.setMotorOutputSettings(TuningConstants.CHAIN_ARM_MOTOR_INVER_OUTPUT, MotorNeutralMode.Brake);
        rightMotor.follow(this.leftMotor);
=======
        this.rightMotor.follow(this.leftMotor);
//==================================== Arm Angle Values ====================================
        this.gearRatio = gearRatio;
        this.minROM = minROM;
        this.maxROM = maxROM;
        this.minROM = TuningConstants.MIN_ROM;
        this.maxROM = TuningConstants.MAX_ROM;
        this.gearRatio = TuningConstants.GEAR_RATIO;
>>>>>>> 08c4c5c65c336c5d82687eccf87fd12fed2a9cd6
    }
    
    @Override
    public void readSensors()
    {
<<<<<<< HEAD
        this.leftMotorPosition = leftMotor.getPosition();
        this.leftMotorVelocity = leftMotor.getVelocity();
=======
        this.leftMotorPosition = leftMotor.Position;
        this.leftMotorVelocity = leftMotor.Velocity;
        this.armAngle = (leftMotor.Position) * this.gearRatio;
>>>>>>> 08c4c5c65c336c5d82687eccf87fd12fed2a9cd6
        this.logger.logNumber(LoggingKey.LeftMotorPosition, this.leftMotorPosition);
        this.logger.logNumber(LoggingKey.LeftMotorVelocity, this.leftMotorVelocity);
        this.logger.logNumber(LoggingKey.ArmAngle, this.armAngle);
    }

    @Override
    public void update()
    {
<<<<<<< HEAD
        double armPower = this.driver.getAnalog(AnalogOperation.MoveChainArm);
        
        this.leftMotor.set(armPower);
=======
        double armPower = this.leftMotor.setPosition(this.driver.getAnalog(AnalogOperation.MoveChainArm));

        if(DigitalOperation.ArmUsePID)
        {
            this.leftMotor.set(TalonXControlMode.Position, armPower);
        }
        
        else if(DigitalOperation.ArmUsePercentOutput)
        {
            this.leftMotor.set(TalonXControlMode.PercentOutput, armPower);
        }
//==================================== Arm Calculations ====================================
        if(this.armAngle > (this.maxROM - 1.0))
        {
            this.leftMotor.set(-0.1);
            wait(500);
            this.leftMotor.set(0);
        }
        else if(this.armAngle < (this.minROM + 1.0))
        {
            this.leftMotor.set(0.1);
            wait(500);
            this.leftMotor.set(0);

        }
>>>>>>> 08c4c5c65c336c5d82687eccf87fd12fed2a9cd6
    }

    @Override
    public void stop()
    {
        this.leftMotor.stop();
    }
}
