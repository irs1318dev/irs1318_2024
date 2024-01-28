package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.*;
import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;

//Inject Functions
import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ArmMechanism implements IMechanism
{
    private static final int defaultPidSlotId = 0;
    private static final int SMPidSlotId = 1;

    private final IDriver driver;
    private final ILogger logger;

    private final ISparkMax shoulderMotor;
    private final ITalonSRX wristMotor;

    private double shoulderPosition;
    private double shoulderVelocity;

    private double wristPosition;
    private double wristVelocity;

    private double theta_3;
    private double theta_1;
    private double theta_4;

    @Inject
    public ArmMechanism(IRobotProvider provider, IDriver driver, ILogger logger)
    {
        this.driver = driver;
        this.logger = logger;

        this.shoulderMotor = provider.getSparkMax(ElectronicsConstants.ARM_SHOULDER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.wristMotor = provider.getTalonSRX(ElectronicsConstants.ARM_WRIST_MOTOR_CAN_ID);

        this.shoulderMotor.setPIDF(
            TuningConstants.ARM_SHOULDER_MOTOR_PID_KP, 
            TuningConstants.ARM_SHOULDER_MOTOR_PID_KI, 
            TuningConstants.ARM_SHOULDER_MOTOR_PID_KD, 
            TuningConstants.ARM_SHOULDER_MOTOR_PID_KF, 
            ArmMechanism.defaultPidSlotId);

        this.wristMotor.setPIDF(
            TuningConstants.ARM_WRIST_MOTOR_PID_KP, 
            TuningConstants.ARM_WRIST_MOTOR_PID_KI, 
            TuningConstants.ARM_WRIST_MOTOR_PID_KD, 
            TuningConstants.ARM_WRIST_MOTOR_PID_KF, 
            ArmMechanism.defaultPidSlotId);

        this.shoulderMotor.setRelativeEncoder();
        // this.shoulderMotor.setInvertSensor(TuningConstants.ARM_SHOULDER_MOTOR_INVERT_SENSOR);
        this.shoulderMotor.setPositionConversionFactor(HardwareConstants.ARM_SHOULDER_TICK_DISTANCE);
        this.shoulderMotor.setInvertOutput(TuningConstants.ARM_SHOULDER_MOTOR_INVERT_OUTPUT);
        this.shoulderMotor.setPosition(TuningConstants.ARM_SHOULDER_STARTING_CONFIGURATION_POSITION);
        this.shoulderMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.shoulderMotor.setSelectedSlot(ArmMechanism.defaultPidSlotId);

        if (TuningConstants.ARM_SHOULDER_USE_PERCENT_OUTPUT)
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.PercentOutput);
        }
        else
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.Position);
        }

        this.shoulderMotor.burnFlash();

        this.wristMotor.setSelectedSlot(ArmMechanism.defaultPidSlotId);
        this.wristMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.wristMotor.setPosition(TuningConstants.ARM_WRIST_STARTING_CONFIGURATION_POSITION);
        this.wristMotor.setMotorOutputSettings(TuningConstants.ARM_WRIST_MOTOR_INVER_OUTPUT, MotorNeutralMode.Brake);
        this.wristMotor.setControlMode(TalonSRXControlMode.PercentOutput);

        ISparkMax shoulderFollowerMotor = provider.getSparkMax(ElectronicsConstants.ARM_SHOULDER_FOLLOWER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        shoulderFollowerMotor.setInvertOutput(TuningConstants.ARM_SHOULDER_MOTOR_FOLLOWER_INVERT_OUTPUT);
        shoulderFollowerMotor.setNeutralMode(MotorNeutralMode.Brake);
        shoulderFollowerMotor.follow(this.shoulderMotor);
        shoulderFollowerMotor.burnFlash();
    }
    
    @Override
    public void readSensors()
    {
        this.shoulderPosition = shoulderMotor.getPosition();
        this.shoulderVelocity = shoulderMotor.getVelocity();
        this.wristPosition = wristMotor.getPosition() * HardwareConstants.ARM_WRIST_TICK_DISTANCE;
        this.wristVelocity = wristMotor.getVelocity();

        this.logger.logNumber(LoggingKey.ArmShoulderPosition, this.shoulderPosition);
        this.logger.logNumber(LoggingKey.ArmShoulderVelocity, this.shoulderVelocity);
        this.logger.logNumber(LoggingKey.ArmWristPosition, this.wristPosition);
        this.logger.logNumber(LoggingKey.ArmWristVelocity, this.wristVelocity);
    }

    @Override
    public void update()
    {
        double armPositionAdjustment = this.driver.getAnalog(AnalogOperation.ArmShoulderPosition);
        double armPowerAdjustment = this.driver.getAnalog(AnalogOperation.ArmShoulderPower);

        double wristPositionAdjustment = this.driver.getAnalog(AnalogOperation.ArmWristPosition);
        double wristPowerAdjustment = this.driver.getAnalog(AnalogOperation.ArmWristPower);

        if (this.driver.getDigital(DigitalOperation.ArmShoulderUsePID))
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.Position);
            this.shoulderMotor.set(armPositionAdjustment);
        }
        else if (this.driver.getDigital(DigitalOperation.ArmShoulderUsePower))
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            this.shoulderMotor.set(armPowerAdjustment);
        }

        if (this.driver.getDigital(DigitalOperation.ArmWristUsePID))
        {
            this.wristMotor.setControlMode(TalonSRXControlMode.Position);
            this.wristMotor.set(wristPositionAdjustment);
        }
        else if (this.driver.getDigital(DigitalOperation.ArmWristUsePower)) 
        {
            this.wristMotor.setControlMode(TalonSRXControlMode.PercentOutput);
            this.wristMotor.set(wristPowerAdjustment);
        }
    }

    private double[] angleToOffsetIKConversion(double armAngle, double wristAngle) 
    {
        double X1_Offset = HardwareConstants.ARM_UPPER_LENGTH * Math.cos(armAngle);
        double Y1_Offset = HardwareConstants.ARM_UPPER_LENGTH * Math.sin(armAngle);
        theta_3 = 180 - armAngle;
        theta_4 = 360 - wristAngle - theta_3;
        double X2_Offset = Math.cos(theta_4) * HardwareConstants.ARM_LOWER_LENGTH;
        double Y2_Offset = Math.sin(theta_4) * HardwareConstants.ARM_LOWER_LENGTH;

        double[] absOffset = {
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
        this.shoulderMotor.stop();
        this.wristMotor.stop();
    }
}
