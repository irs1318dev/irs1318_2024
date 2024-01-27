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
import frc.robot.driver.controltasks.WaitTask;

//Inject Functions
import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ChainArmMechanism implements IMechanism{

    private static final int defaultPidSlotId = 0;

    private final IDriver driver;
    private final ILogger logger;
    private double chainMotorPosition;
    private double chainMotorVelocity;
    private final ITalonSRX chainMotor;
    private double gearRatio;
    /* 
    private double minAngle;
    private double maxAngle;
    These can be accessed by tuning constants CHAIN_ARM_MIN_ANGLE and CHAIN_ARM_MAX_ANGLE
    */

    private double armAngle;
    

    private final ITalonSRX wristMotor;
    private double wristMotorPosition;
    private double wristMotorVelocity;

    @Inject
    public ChainArmMechanism(IRobotProvider provider, IDriver driver, ILogger logger)
    {
        this.driver = driver;
        this.logger = logger;
        this.

        this.chainMotor = provider.getTalonSRX(ElectronicsConstants.LEFT_CHAIN_MOTOR_CAN_ID);
        this.wristMotor = provider.getTalonSRX(ElectronicsConstants.WRIST_CHAIN_MOTOR_CAN_ID);

        this.chainMotor.setPIDF(
            TuningConstants.CHAIN_ARM_MOTOR_PID_KP, 
            TuningConstants.CHAIN_ARM_MOTOR_PID_KI, 
            TuningConstants.CHAIN_ARM_MOTOR_PID_KD, 
            TuningConstants.CHAIN_ARM_MOTOR_PID_KF, 
            ChainArmMechanism.defaultPidSlotId);
        
        this.wristMotor.setPIDF(
            TuningConstants.WRIST_ARM_MOTOR_PID_KP, 
            TuningConstants.WRIST_ARM_MOTOR_PID_KI, 
            TuningConstants.WRIST_ARM_MOTOR_PID_KD, 
            TuningConstants.WRIST_ARM_MOTOR_PID_KF, 
            ChainArmMechanism.defaultPidSlotId);

        this.chainMotor.setSelectedSlot(defaultPidSlotId);
        this.chainMotor.setSensorType(TalonSRXFeedbackDevice.None);
        this.chainMotor.setPosition(0.0);
        this.chainMotor.setMotorOutputSettings(TuningConstants.CHAIN_ARM_MOTOR_INVER_OUTPUT, MotorNeutralMode.Brake);
        this.chainMotor.setControlMode(TalonSRXControlMode.PercentOutput);

        this.wristMotor.setSelectedSlot(defaultPidSlotId);
        this.wristMotor.setSensorType(TalonSRXFeedbackDevice.None);
        this.wristMotor.setPosition(0.0);
        this.wristMotor.setMotorOutputSettings(TuningConstants.WRIST_ARM_MOTOR_INVER_OUTPUT, MotorNeutralMode.Brake);
        this.wristMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        

        ITalonSRX rightMotor = provider.getTalonSRX(ElectronicsConstants.RIGHT_CHAIN_MOTOR_CAN_ID);
        rightMotor.setMotorOutputSettings(TuningConstants.CHAIN_ARM_MOTOR_INVER_OUTPUT, MotorNeutralMode.Brake);
<<<<<<< Updated upstream
        rightMotor.follow(this.leftMotor);
        rightMotor.follow(this.leftMotor);

=======
        rightMotor.follow(this.chainMotor);
        rightMotor.follow(this.chainMotor);
>>>>>>> Stashed changes
    }
    
    @Override
    public void readSensors()
    {
<<<<<<< Updated upstream
        this.leftMotorPosition = leftMotor.getPosition();
        this.leftMotorVelocity = leftMotor.getVelocity();
        this.armAngle = (leftMotorPosition) * HardwareConstants.CHAINARM_TICKS_TO_ANGLE;
        this.logger.logNumber(LoggingKey.LeftMotorPosition, this.leftMotorPosition);
        this.logger.logNumber(LoggingKey.LeftMotorVelocity, this.leftMotorVelocity);
        this.logger.logNumber(LoggingKey.ArmAngle, this.armAngle);
=======
        this.chainMotorPosition = chainMotor.getPosition();
        this.chainMotorVelocity = chainMotor.getVelocity();
        this.wristMotorPosition = wristMotor.getPosition();
        this.wristMotorVelocity = wristMotor.getVelocity();

        this.logger.logNumber(LoggingKey.ChainMotorPosition, this.chainMotorPosition);
        this.logger.logNumber(LoggingKey.ChainMotorVelocity, this.chainMotorVelocity);
        this.logger.logNumber(LoggingKey.WristMotorPosition, this.wristMotorPosition);
        this.logger.logNumber(LoggingKey.WristMotorVelocity, this.wristMotorVelocity);
>>>>>>> Stashed changes
    }

    @Override
    public void update()
    {
        double armPositionAdjustment = this.driver.getAnalog(AnalogOperation.PositionChainArm);
        double armPowerAdjustment = this.driver.getAnalog(AnalogOperation.PowerChainArm);

        double wristPositionAdjustment = this.driver.getAnalog(AnalogOperation.WristPositionChainArm);
        double wristPowerAdjustment = this.driver.getAnalog(AnalogOperation.WristPowerChainArm);

        if(this.driver.getDigital(DigitalOperation.ArmUsePID))
        {
            this.chainMotor.set(TalonSRXControlMode.Position, armPositionAdjustment);
        }
        
        else if(this.driver.getDigital(DigitalOperation.ArmUsePercentOutput))
        {
            this.chainMotor.set(TalonSRXControlMode.PercentOutput, armPowerAdjustment);
        }
<<<<<<< Updated upstream
//==================================== Arm Calculations ====================================

        /*
        if(this.armAngle > (TuningConstants.CHAINARM_MAX_ANGLE - 1.0))
=======
        
        if(this.driver.getDigital(DigitalOperation.WristUsePID))
>>>>>>> Stashed changes
        {
            this.chainMotor.set(TalonSRXControlMode.PercentOutput, wristPositionAdjustment);
        }
<<<<<<< Updated upstream
        else if(this.armAngle < (TuningConstants.CHAINARM_MIN_ANGLE + 1.0))
        {
            this.leftMotor.set(0.1);
            new WaitTask(500);
            this.leftMotor.set(0);
=======
>>>>>>> Stashed changes

        else if(this.driver.getDigital(DigitalOperation.WristUsePower)) 
        {
            this.chainMotor.set(TalonSRXControlMode.Position, wristPowerAdjustment);
        }

        TODO: This is supposed to make sure the arm is not overextending. Wrong use of waittask.

        */
    }

    @Override
    public void stop()
    {
        this.chainMotor.stop();
        this.wristMotor.stop();
    }
}
