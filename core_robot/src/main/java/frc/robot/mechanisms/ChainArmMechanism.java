package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ISparkMax;
import frc.lib.robotprovider.ITalonFX;
import frc.lib.robotprovider.ITalonSRX;
import frc.lib.robotprovider.MotorNeutralMode;
import frc.lib.robotprovider.SparkMaxControlMode;
import frc.lib.robotprovider.SparkMaxMotorType;
import frc.lib.robotprovider.TalonSRXControlMode;
import frc.lib.robotprovider.TalonSRXFeedbackDevice;
import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
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
    private static final int SMPidSlotId = 1;

    private final IDriver driver;
    private final ILogger logger;

    private double chainMotorPosition;
    private double chainMotorVelocity;

    private final ISparkMax chainMotor;

    private double armGearRatio;
    private double wristGearRatio;
    
    private final ITalonSRX wristMotor;
    private double wristMotorPosition;
    private double wristMotorVelocity;

    private double theta_3;
    private double theta_1;
    private double theta_4;

    @Inject
    public ChainArmMechanism(IRobotProvider provider, IDriver driver, ILogger logger)
    {
        this.driver = driver;
        this.logger = logger;

        this.armGearRatio= HardwareConstants.CHAINARM_TICKS_TO_ANGLE;
        this.wristGearRatio = HardwareConstants.CHAIN_WRIST_TICKS_TO_ANGLE;

        this.chainMotor = provider.getSparkMax(ElectronicsConstants.CHAIN_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
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

        this.chainMotor.setRelativeEncoder();
        this.chainMotor.setInvertSensor(TuningConstants.CHAIN_MOTOR_INVERT_SENSOR);
        this.chainMotor.setPositionConversionFactor(this.armGearRatio);
        this.chainMotor.setInvertOutput(TuningConstants.CHAIN_MOTOR_INVERT_OUTPUT);
        this.chainMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.chainMotor.setSelectedSlot(defaultPidSlotId);

        if (TuningConstants.SHOULDER_USE_PERCENT_OUTPUT)
        {
            this.chainMotor.setControlMode(SparkMaxControlMode.PercentOutput);
        }

        else
        {
            this.chainMotor.setControlMode(SparkMaxControlMode.Position);
        }

        this.chainMotor.burnFlash();

        this.wristMotor.setSelectedSlot(defaultPidSlotId);
        this.wristMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.wristMotor.setPosition(0.0);
        this.wristMotor.setPositionConversionFactor(this.wristGearRatio);
        this.wristMotor.setMotorOutputSettings(TuningConstants.WRIST_ARM_MOTOR_INVER_OUTPUT, MotorNeutralMode.Brake);
        this.wristMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        this.wristMotor.burnFlash();
        

        ISparkMax rightMotor = provider.getSparkMax(ElectronicsConstants.CHAIN_FOLLOWER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        rightMotor.setInvertOutput(TuningConstants.CHAIN_MOTOR_FOLLOWER_INVERT_OUTPUT);
        rightMotor.setNeutralMode(MotorNeutralMode.Brake);
        rightMotor.follow(this.chainMotor);
        rightMotor.burnFlash();

    }
    
    @Override
    public void readSensors()
    {
        this.chainMotorPosition = chainMotor.getPosition() * armGearRatio;
        this.chainMotorVelocity = chainMotor.getVelocity();
        this.wristMotorPosition = wristMotor.getPosition() * wristGearRatio;
        this.wristMotorVelocity = wristMotor.getVelocity();

        this.logger.logNumber(LoggingKey.ChainMotorPosition, this.chainMotorPosition);
        this.logger.logNumber(LoggingKey.ChainMotorVelocity, this.chainMotorVelocity);
        this.logger.logNumber(LoggingKey.WristMotorPosition, this.wristMotorPosition);
        this.logger.logNumber(LoggingKey.WristMotorVelocity, this.wristMotorVelocity);
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
            this.chainMotor.setControlMode(SparkMaxControlMode.Position);
            this.chainMotor.set(armPositionAdjustment);
        }
        
        else if(this.driver.getDigital(DigitalOperation.ArmUsePower))
        {
            this.chainMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            this.chainMotor.set(armPowerAdjustment);
        }
        
        if(this.driver.getDigital(DigitalOperation.WristUsePID))
        {
            this.wristMotor.setControlMode(TalonSRXControlMode.Position);
            this.wristMotor.set(wristPositionAdjustment);
        }

        else if(this.driver.getDigital(DigitalOperation.WristUsePower)) 
        {
            this.wristMotor.setControlMode(TalonSRXControlMode.PercentOutput);
            this.wristMotor.set(wristPowerAdjustment);
        }
    }

    private double[] angleToOffsetIKConversion(double armAngle, double wristAngle) 
    {
        double X1_Offset = HardwareConstants.CHAIN_ARM_LENGTH * Math.cos(armAngle);
        double Y1_Offset = HardwareConstants.CHAIN_ARM_LENGTH * Math.sin(armAngle);
        theta_3 = 180 - armAngle;
        theta_4 = 360 - wristAngle - theta_3;
        double X2_Offset = Math.cos(theta_4) * HardwareConstants.CHAIN_ARM_WRIST_LENGTH;
        double Y2_Offset = Math.sin(theta_4) * HardwareConstants.CHAIN_ARM_WRIST_LENGTH;

        double[][] absOffset = {
            X2_Offset + X1_Offset + HardwareConstants.CAMERA_TO_ARM_X_OFFSET, // May need to subtract camera offset instead
            Y2_Offset + Y2_Offset + HardwareConstants.CAMERA_TO_ARM_Y_OFFSET};
        return absOffset;
    }

    public double getTheta3()
    {
        return this.theta_3;
    }
    
    public double getTheta1()
    {
        return this.theta_1;
    }

    public double getTheta4()
    {
        return this.theta_4;
    }

    @Override
    public void stop()
    {
        this.chainMotor.stop();
        this.wristMotor.stop();
    }
}
