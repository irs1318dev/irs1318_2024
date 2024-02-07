package frc.robot.mechanisms;

import frc.lib.controllers.TrapezoidProfile;
import frc.lib.driver.IDriver;
import frc.lib.filters.FloatingAverageCalculator;
import frc.lib.helpers.Helpers;
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
    private static final int DefaultPidSlotId = 0;
    private static final int AltPidSlotId = 1;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;
    private final PowerManager powerManager;

    private double prevTime;

    private final ISparkMax shoulderMotor;
    // private final ITalonSRX wristMotor;

    private double shoulderPosition;
    private double shoulderVelocity;

    private double desiredShoulderPosition;
    private double desiredWristPosition;

    private double wristPosition;
    private double wristVelocity;

    private double shoulderError;
    private double wristError;

    private double shoulderSetpointChangedTime;
    private double wristSetpointChangedTime;
    private boolean shoulderStalled;
    private boolean wristStalled;

    private FloatingAverageCalculator shoulderPowerAverageCalculator;
    private FloatingAverageCalculator wristPowerAverageCalculator;

    private double shoulderPowerAverage;
    private double wristPowerAverage;

    private FloatingAverageCalculator shoulderVelocityAverageCalculator;
    private FloatingAverageCalculator wristVelocityAverageCalculator;

    private double shoulderVelocityAverage;
    private double wristVelocityAverage;

    private final TrapezoidProfile shoulderTrapezoidMotionProfile;
    private final TrapezoidProfile.State shoulderTMPCurrState;
    private final TrapezoidProfile.State shoulderTMPGoalState;

    private double theta_1;
    private double theta_3;
    private double theta_4;

    private double armAngle;
    private double wristAngle;

    private double shooterXOffset;
    private double shooterZOffset;

    private boolean inSimpleMode;

    @Inject
    public ArmMechanism(IRobotProvider provider, IDriver driver, LoggingManager logger, ITimer timer, PowerManager powerManager)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;
        this.powerManager = powerManager;

        this.inSimpleMode = TuningConstants.ARM_USE_SIMPLE_MODE;

        this.shoulderMotor = provider.getSparkMax(ElectronicsConstants.ARM_SHOULDER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        // this.wristMotor = provider.getTalonSRX(ElectronicsConstants.ARM_WRIST_MOTOR_CAN_ID);

        this.shoulderMotor.setRelativeEncoder();
        // this.shoulderMotor.setInvertSensor(TuningConstants.ARM_SHOULDER_MOTOR_INVERT_SENSOR);
        this.shoulderMotor.setPositionConversionFactor(HardwareConstants.ARM_SHOULDER_TICK_DISTANCE);
        this.shoulderMotor.setVelocityConversionFactor(HardwareConstants.ARM_SHOULDER_TICK_DISTANCE);
        this.shoulderMotor.setInvertOutput(TuningConstants.ARM_SHOULDER_MOTOR_INVERT_OUTPUT);
        this.shoulderMotor.setPosition(TuningConstants.ARM_SHOULDER_STARTING_CONFIGURATION_POSITION);
        this.shoulderMotor.setNeutralMode(MotorNeutralMode.Brake);

        // this.wristMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        // this.wristMotor.setPosition(TuningConstants.ARM_WRIST_STARTING_CONFIGURATION_POSITION);
        // this.wristMotor.setMotorOutputSettings(TuningConstants.ARM_WRIST_MOTOR_INVER_OUTPUT, MotorNeutralMode.Brake);

        if (TuningConstants.ARM_USE_MM)
        {
            this.shoulderMotor.setPIDF(
                TuningConstants.ARM_SHOULDER_POSITION_TMP_PID_KP,
                TuningConstants.ARM_SHOULDER_POSITION_TMP_PID_KI,
                TuningConstants.ARM_SHOULDER_POSITION_TMP_PID_KD,
                TuningConstants.ARM_SHOULDER_POSITION_TMP_PID_KF,
                ArmMechanism.AltPidSlotId);

            // this.wristMotor.setMotionMagicPIDF(
            //     TuningConstants.ARM_WRIST_POSITION_MM_PID_KP,
            //     TuningConstants.ARM_WRIST_POSITION_MM_PID_KI,
            //     TuningConstants.ARM_WRIST_POSITION_MM_PID_KD,
            //     TuningConstants.ARM_WRIST_POSITION_MM_PID_KF,
            //     TuningConstants.ARM_WRIST_POSITION_MM_CRUISE_VELOCITY,
            //     TuningConstants.ARM_WRIST_POSITION_MM_ACCELERATION,
            //     ArmMechanism.AltPidSlotId);
        }
        else
        {
            this.shoulderMotor.setPIDF(
                TuningConstants.ARM_SHOULDER_MOTOR_PID_KP,
                TuningConstants.ARM_SHOULDER_MOTOR_PID_KI,
                TuningConstants.ARM_SHOULDER_MOTOR_PID_KD,
                TuningConstants.ARM_SHOULDER_MOTOR_PID_KF,
                ArmMechanism.DefaultPidSlotId);

            // this.wristMotor.setPIDF(
            //     TuningConstants.ARM_WRIST_MOTOR_PID_KP,
            //     TuningConstants.ARM_WRIST_MOTOR_PID_KI,
            //     TuningConstants.ARM_WRIST_MOTOR_PID_KD,
            //     TuningConstants.ARM_WRIST_MOTOR_PID_KF,
            //     ArmMechanism.DefaultPidSlotId);
        }

        if (TuningConstants.ARM_USE_MM)
        {
            this.shoulderTrapezoidMotionProfile = new TrapezoidProfile(
                TuningConstants.ARM_SHOULDER_TMP_PID_CRUISE_VELOC,
                TuningConstants.ARM_SHOULDER_TMP_PID_ACCEL);

            this.shoulderTMPCurrState = new TrapezoidProfile.State(0.0, 0.0);
            this.shoulderTMPGoalState = new TrapezoidProfile.State(0.0, 0.0);

            this.shoulderMotor.setSelectedSlot(ArmMechanism.AltPidSlotId);
            // this.wristMotor.setSelectedSlot(ArmMechanism.AltPidSlotId);
        }
        else
        {
            this.shoulderTrapezoidMotionProfile = null;
            this.shoulderTMPCurrState = null;
            this.shoulderTMPGoalState = null;

            this.shoulderMotor.setSelectedSlot(ArmMechanism.DefaultPidSlotId);
            // this.wristMotor.setSelectedSlot(ArmMechanism.DefaultPidSlotId);
        }

        if (this.inSimpleMode)
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            // this.wristMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        }
        else if (TuningConstants.ARM_USE_MM)
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.Position);
            // this.wristMotor.setControlMode(TalonSRXControlMode.MotionMagicPosition);
        }
        else
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.Position);
            // this.wristMotor.setControlMode(TalonSRXControlMode.Position);
        }

        this.shoulderMotor.burnFlash();

        ISparkMax shoulderFollowerMotor = provider.getSparkMax(ElectronicsConstants.ARM_SHOULDER_FOLLOWER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        shoulderFollowerMotor.setInvertOutput(TuningConstants.ARM_SHOULDER_MOTOR_FOLLOWER_INVERT_OUTPUT);
        shoulderFollowerMotor.setNeutralMode(MotorNeutralMode.Brake);
        shoulderFollowerMotor.follow(this.shoulderMotor);
        shoulderFollowerMotor.burnFlash();

        this.shoulderPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_SHOULDER_POWER_TRACKING_DURATION, TuningConstants.ARM_SHOULDER_POWER_SAMPLES_PER_SECOND);
        this.wristPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_WRIST_POWER_TRACKING_DURATION, TuningConstants.ARM_WRIST_POWER_SAMPLES_PER_SECOND);

        this.shoulderVelocityAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_SHOULDER_VELOCITY_TRACKING_DURATION, TuningConstants.ARM_SHOULDER_VELOCITY_SAMPLES_PER_SECOND);
        this.wristVelocityAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_WRIST_VELOCITY_TRACKING_DURATION, TuningConstants.ARM_WRIST_VELOCITY_SAMPLES_PER_SECOND);

        this.shooterXOffset = HardwareConstants.ARM_SHOOTER_RETRACT_X_POS;
        this.shooterZOffset = HardwareConstants.ARM_SHOOTER_RETRACT_Z_POS;
    }

    @Override
    public void readSensors()
    {
        this.shoulderPosition = this.shoulderMotor.getPosition(); // in degrees (conversion to degrees included in setPositionConversionFactor)
        this.shoulderVelocity = this.shoulderMotor.getVelocity(); // in degrees/sec (conversion to degrees included in setVelocityConversionFactor)
        this.shoulderError = this.shoulderPosition - this.desiredShoulderPosition;
        // this.wristPosition = this.wristMotor.getPosition() * HardwareConstants.ARM_WRIST_TICK_DISTANCE; // convert rotations to degrees
        // this.wristVelocity = this.wristMotor.getVelocity() * HardwareConstants.ARM_WRIST_TICK_DISTANCE; // convert rotations/sec to degrees/sec
        // this.wristError = this.wristMotor.getError();

        this.angleToShooterOffsetFK(this.armAngle, this.wristAngle);

        double shoulderCurrent = this.powerManager.getCurrent(ElectronicsConstants.ARM_SHOULDER_PDH_CHANNEL);
        double shoulderFollowerCurrent = this.powerManager.getCurrent(ElectronicsConstants.ARM_SHOULDER_FOLLOWER_PDH_CHANNEL);
        double wristCurrent = this.powerManager.getCurrent(ElectronicsConstants.ARM_WRIST_PDH_CHANNEL);
        double batteryVoltage = this.powerManager.getBatteryVoltage();

        this.shoulderPowerAverage = this.shoulderPowerAverageCalculator.update(((shoulderCurrent + shoulderFollowerCurrent) * 0.5) * batteryVoltage);
        this.wristPowerAverage = this.wristPowerAverageCalculator.update(wristCurrent * batteryVoltage);

        this.shoulderVelocityAverage = this.shoulderVelocityAverageCalculator.update(Math.abs(this.shoulderVelocity));
        this.wristVelocityAverage = this.wristVelocityAverageCalculator.update(Math.abs(this.wristVelocity));

        this.logger.logNumber(LoggingKey.ArmShoulderPosition, this.shoulderPosition);
        this.logger.logNumber(LoggingKey.ArmShoulderVelocity, this.shoulderVelocity);
        this.logger.logNumber(LoggingKey.ArmShoulderVelocityAverage, this.shoulderVelocityAverage);
        this.logger.logNumber(LoggingKey.ArmShoulderError, this.shoulderError);
        this.logger.logNumber(LoggingKey.ArmShoulderPowerAverage, this.shoulderPowerAverage);
        this.logger.logNumber(LoggingKey.ArmWristPosition, this.wristPosition);
        this.logger.logNumber(LoggingKey.ArmWristVelocity, this.wristVelocity);
        this.logger.logNumber(LoggingKey.ArmWristVelocityAverage, this.wristVelocityAverage);
        this.logger.logNumber(LoggingKey.ArmWristError, this.wristError);
        this.logger.logNumber(LoggingKey.ArmWristPowerAverage, this.wristPowerAverage);
    }

    @Override
    public void update()
    {
        double currTime = this.timer.get();

        if (!this.inSimpleMode && this.driver.getDigital(DigitalOperation.ArmEnableSimpleMode))
        {
            this.inSimpleMode = true;
        }
        else if (this.inSimpleMode && this.driver.getDigital(DigitalOperation.ArmDisableSimpleMode))
        {
            this.inSimpleMode = false;

            if (TuningConstants.ARM_USE_MM)
            {
                this.shoulderMotor.setSelectedSlot(ArmMechanism.AltPidSlotId);
                // this.wristMotor.setSelectedSlot(ArmMechanism.AltPidSlotId);
            }
            else
            {
                this.shoulderMotor.setSelectedSlot(ArmMechanism.DefaultPidSlotId);
                // this.wristMotor.setSelectedSlot(ArmMechanism.DefaultPidSlotId);
            }

            this.desiredShoulderPosition = this.shoulderPosition;
            this.desiredWristPosition = this.wristPosition;

            this.shoulderSetpointChangedTime = currTime;
            this.wristSetpointChangedTime = currTime;

            this.shoulderStalled = false;
            this.wristStalled = false;
        }

        // ---------------> MAIN ARM CONTROL <------------------------

        double shoulderPowerAdjustment = this.driver.getAnalog(AnalogOperation.ArmShoulderPower);
        double wristPowerAdjustment = this.driver.getAnalog(AnalogOperation.ArmWristPower);

        double shoulderPower = 0.0;
        double wristPower = 0.0;
        boolean useSimpleMode = false;

        if (this.inSimpleMode)
        {
            useSimpleMode = true;
            shoulderPower = shoulderPowerAdjustment;
            wristPower = wristPowerAdjustment;

            this.shoulderSetpointChangedTime = currTime;
            this.wristSetpointChangedTime = currTime;


            this.shoulderStalled = false;
            this.wristStalled = false;
        }
        else
        {
            if (shoulderPowerAdjustment != 0.0 || wristPowerAdjustment != 0.0)
            {
                useSimpleMode = true;

                if (shoulderPowerAdjustment != 0.0)
                {
                    // Updated desired to current position
                    this.desiredShoulderPosition = this.shoulderPosition;

                    // Update changed time and stalled constants
                    this.shoulderSetpointChangedTime = currTime;
                    this.shoulderStalled = false;

                    // Update this power value
                    shoulderPower = shoulderPowerAdjustment;
                }

                if (wristPowerAdjustment != 0.0)
                {
                    // Updated desired to current position
                    this.desiredWristPosition = this.wristPosition;

                    // Update changed time and stalled constants
                    this.wristSetpointChangedTime = currTime;
                    this.wristStalled = false;

                    // Update this power value
                    wristPower = wristPowerAdjustment;
                }

                // potentially add stuff for IK and FK setting
            }
            else
            {
                double newDesiredShoulderPosition = this.driver.getAnalog(AnalogOperation.ArmShoulderPositionSetpoint);
                double newDesiredWristPosition = this.driver.getAnalog(AnalogOperation.ArmWristPositionSetpoint);

                if (newDesiredShoulderPosition != TuningConstants.MAGIC_NULL_VALUE ||
                    newDesiredWristPosition != TuningConstants.MAGIC_NULL_VALUE)
                {
                    // controlled by macro
                    if (newDesiredShoulderPosition != TuningConstants.MAGIC_NULL_VALUE &&
                        !Helpers.RoughEquals(this.desiredShoulderPosition, newDesiredShoulderPosition, 0.1))
                    {
                        this.shoulderSetpointChangedTime = currTime;
                        this.shoulderStalled = false;

                        this.desiredShoulderPosition = newDesiredShoulderPosition;
                    }

                    if (newDesiredWristPosition != TuningConstants.MAGIC_NULL_VALUE &&
                        !Helpers.RoughEquals(this.desiredWristPosition, newDesiredWristPosition, 0.1))
                    {
                        this.wristSetpointChangedTime = currTime;
                        this.wristStalled = false;

                        this.desiredWristPosition = newDesiredWristPosition;
                    }
                }
            }
        }

        if (TuningConstants.ARM_STALL_PROTECTION_ENABLED)
        {
            if (currTime > this.shoulderSetpointChangedTime + TuningConstants.ARM_SHOULDER_VELOCITY_TRACKING_DURATION &&
                this.shoulderPowerAverage >= TuningConstants.ARM_SHOULDER_STALLED_POWER_THRESHOLD &&
                Math.abs(this.shoulderVelocityAverage) <= TuningConstants.ARM_SHOULDER_STALLED_VELOCITY_THRESHOLD)
            {
                this.shoulderStalled = true;
            }

            if (currTime > this.wristSetpointChangedTime + TuningConstants.ARM_WRIST_VELOCITY_TRACKING_DURATION &&
                this.wristPowerAverage >= TuningConstants.ARM_WRIST_STALLED_POWER_THRESHOLD &&
                Math.abs(this.wristVelocityAverage) <= TuningConstants.ARM_WRIST_STALLED_VELOCITY_THRESHOLD)
            {
                this.wristStalled = true;
            }
        }

        if (!useSimpleMode)
        {
            if (this.shoulderStalled)
            {
                this.shoulderMotor.stop();
            }
            else
            {
                double elapsedTime = currTime - this.prevTime;
                this.shoulderMotor.setControlMode(SparkMaxControlMode.Position);

                double actualDesiredShoulderPosition = this.desiredShoulderPosition;

                if (TuningConstants.ARM_USE_MM)
                {
                    TrapezoidProfile.State curr = this.shoulderTMPCurrState;
                    TrapezoidProfile.State goal = this.shoulderTMPGoalState;

                    if (goal.updatePosition(desiredShoulderPosition))
                    {
                        curr.updatePosition(this.shoulderPosition);
                    }

                    if (this.shoulderTrapezoidMotionProfile.update(elapsedTime, curr, goal))
                    {
                        actualDesiredShoulderPosition = curr.getPosition();
                    }
                }

                this.shoulderMotor.set(actualDesiredShoulderPosition);
            }

            if (this.wristStalled)
            {
                // this.wristMotor.stop();
            }
            else
            {
                // this.wristMotor.set(
                //     TuningConstants.ARM_USE_MM ? TalonSRXControlMode.MotionMagicPosition : TalonSRXControlMode.Position,
                //     this.desiredShoulderPosition);
            }
        }
        else
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            this.shoulderMotor.set(shoulderPower);
            // this.wristMotor.set(TalonSRXControlMode.PercentOutput, wristPower);
        }

        this.logger.logBoolean(LoggingKey.ArmShoulderStalled, this.shoulderStalled);
        this.logger.logBoolean(LoggingKey.ArmWristStalled, this.wristStalled);

        this.logger.logNumber(LoggingKey.ArmShoulderSetpoint, this.desiredShoulderPosition);
        this.logger.logNumber(LoggingKey.ArmWristSetpoint, this.desiredWristPosition);

        this.prevTime = currTime;
    }

    @Override
    public void stop()
    {
        this.shoulderMotor.stop();
        // this.wristMotor.stop();
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
}
