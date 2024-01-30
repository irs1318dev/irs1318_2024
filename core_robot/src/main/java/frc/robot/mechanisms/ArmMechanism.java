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

    private double armAngle;
    private double wristAngle;

    private double shooterXOffset;
    private double shooterZOffset;

    private boolean inSimpleMode;

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

        this.inSimpleMode = TuningConstants.ARM_USE_SIMPLE_MODE;

        this.shoulderMotor.burnFlash();

        this.wristMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.wristMotor.setPosition(TuningConstants.ARM_WRIST_STARTING_CONFIGURATION_POSITION);
        this.wristMotor.setMotorOutputSettings(TuningConstants.ARM_WRIST_MOTOR_INVER_OUTPUT, MotorNeutralMode.Brake);
        this.wristMotor.setControlMode(TalonSRXControlMode.PercentOutput);

        ISparkMax shoulderFollowerMotor = provider.getSparkMax(ElectronicsConstants.ARM_SHOULDER_FOLLOWER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        shoulderFollowerMotor.setInvertOutput(TuningConstants.ARM_SHOULDER_MOTOR_FOLLOWER_INVERT_OUTPUT);
        shoulderFollowerMotor.setNeutralMode(MotorNeutralMode.Brake);
        shoulderFollowerMotor.follow(this.shoulderMotor);
        shoulderFollowerMotor.burnFlash();

        this.shooterXOffset = HardwareConstants.ARM_SHOOTER_RETRACT_X_POS;
        this.shooterZOffset = HardwareConstants.ARM_SHOOTER_RETRACT_Z_POS;
    }
    
    @Override
    public void readSensors()
    {
        this.shoulderPosition = shoulderMotor.getPosition() * HardwareConstants.ARM_SHOULDER_TICK_DISTANCE;
        this.shoulderVelocity = shoulderMotor.getVelocity();
        this.wristPosition = wristMotor.getPosition() * HardwareConstants.ARM_WRIST_TICK_DISTANCE;
        this.wristVelocity = wristMotor.getVelocity();

        this.angleToShooterOffsetFK(this.armAngle, this.wristAngle);

        this.logger.logNumber(LoggingKey.ArmShoulderPosition, this.shoulderPosition);
        this.logger.logNumber(LoggingKey.ArmShoulderVelocity, this.shoulderVelocity);
        this.logger.logNumber(LoggingKey.ArmWristPosition, this.wristPosition);
        this.logger.logNumber(LoggingKey.ArmWristVelocity, this.wristVelocity);
    }

    @Override
    public void update()
    {
        if (this.driver.getDigital(DigitalOperation.ArmEnableSimpleMode))
        {
            this.inSimpleMode = true;
        }
        else if(this.driver.getDigital(DigitalOperation.ArmDisableSimpleMode))
        {
            this.inSimpleMode = false;
        }

        double armPositionAdjustment = this.driver.getAnalog(AnalogOperation.ArmShoulderPosition);
        double wristPositionAdjustment = this.driver.getAnalog(AnalogOperation.ArmWristPosition);

        double armPower = this.shoulderPosition;
        double wristPower = this.wristPosition;

        boolean useSimpleMode = false;

        if (this.inSimpleMode)
        {
            useSimpleMode = true;
            armPower = armPositionAdjustment;
            wristPower = wristPositionAdjustment;
        }
        else 
        {
            if (armPositionAdjustment != 0 || wristPositionAdjustment != 0) 
            {
                useSimpleMode = true;
                armPower = armPositionAdjustment;
                wristPower = wristPositionAdjustment;
            }
            else
            {
                if (armPositionAdjustment != TuningConstants.MAGIC_NULL_VALUE
                    && wristPositionAdjustment != TuningConstants.MAGIC_NULL_VALUE)
                {
                    armPower = armPositionAdjustment;
                    wristPower = wristPositionAdjustment;
                    useSimpleMode = false;
                }
            }
        }

        if (useSimpleMode)
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            this.shoulderMotor.set(armPower);
            this.wristMotor.set(TalonSRXControlMode.PercentOutput, wristPower);
        }
        else 
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.Position);
            this.shoulderMotor.set(armPower);
            this.wristMotor.set(TalonSRXControlMode.Position, wristPower);
        }
    }

    private void angleToShooterOffsetFK(double armAngle, double wristAngle) 
    {
        double X1_Offset = HardwareConstants.ARM_HUMERUS_LENGTH * Math.cos(armAngle);
        double Z1_Offset = HardwareConstants.ARM_HUMERUS_LENGTH * Math.sin(armAngle);
        theta_3 = 180 - armAngle;
        theta_4 = 360 - wristAngle - theta_3;
        double X2_Offset = Math.cos(theta_4) * HardwareConstants.ARM_ULNA_LENGTH;
        double Z2_Offset = Math.sin(theta_4) * HardwareConstants.ARM_ULNA_LENGTH;

        this.shooterXOffset = X2_Offset + X1_Offset + HardwareConstants.CAMERA_TO_ARM_X_OFFSET;
        this.shooterZOffset = Z2_Offset + Z1_Offset + HardwareConstants.CAMERA_TO_ARM_Z_OFFSET;
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

    public double getXOffset()
    {
        return this.shooterXOffset;
    }
    
    public double getZOffset()
    {
        return this.shooterZOffset;
    }

    @Override
    public void stop()
    {
        this.shoulderMotor.stop();
        this.wristMotor.stop();
    }
}
