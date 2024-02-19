package frc.robot.mechanisms;

import frc.lib.controllers.TrapezoidProfile;
import frc.lib.driver.IDriver;
import frc.lib.filters.FloatingAverageCalculator;
import frc.lib.helpers.BilinearInterpolator;
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
    private final ITalonSRX wristMotor;

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

    // Gravity Compensation
    private final BilinearInterpolator interpolator;

    // IK VARIABLES
    private double theta_1; // Horizontal to shoulder ABS angle
    private double theta_2; // Wrist to Shoulder rel angle
    private double theta_3; // 180 - theta_1
    private double theta_4; // Horizontal to Wrist (shooter bottom) ABS angle
    private double theta_5; // Horizontal to intake bottom ABS angle
    private double theta_6 = HardwareConstants.SHOOTER_TRIANGLE_ANGLE; // Shooter triangle angle
    private double theta_7 = HardwareConstants.INTAKE_TRIANGLE_ANGLE; // Intake triangle angle
    private double theta_8; // Horizontal to shooter top ABS angle
    private double theta_9; // Horizontal to intake top ABS angle

    private final double L1 = HardwareConstants.ARM_HUMERUS_LENGTH; // Shoulder pivot to wrist pivot distance
    private final double L2 = HardwareConstants.ARM_WRIST_TO_SHOOTER_EDGE; // wrist to shooter top
    private final double L3 = HardwareConstants.ARM_WRIST_TO_INTAKE_EDGE; // wirst to intake top
    private final double L2x = HardwareConstants.ARM_WRIST_TO_SHOOTER_X; // wrist to shooter bottom
    private final double L2z = HardwareConstants.ARM_WRIST_TO_INTAKE_Z; // intake height
    private final double L3x = HardwareConstants.ARM_WRIST_TO_INTAKE_X; // wrist to intake bottom
    private final double L3z = HardwareConstants.ARM_WRIST_TO_SHOOTER_Z; // shooter height

    private double shoulderJointAbsPosX = HardwareConstants.ARM_TO_CENTER_ROBOT_X_OFFSET;
    private double shoulderJointAbsPosZ = HardwareConstants.ARM_TO_CENTER_ROBOT_Z_OFFSET;
    private double wristAbsPosX = this.shoulderJointAbsPosX + Helpers.cosd(this.theta_1) * L1;
    private double wristAbsPosZ = this.shoulderJointAbsPosZ + Helpers.sind(this.theta_1) * L1;
    private double shooterBottomAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_4) * L2x;
    private double shooterBottomAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_4) * L2x;
    private double shooterTopAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_8) * L2;
    private double shooterTopAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_8) * L2;
    private double intakeBottomAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_5) * L3x;
    private double intakeBottomAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_5) * L3x;
    private double intakeTopAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_9) * L3;
    private double intakeTopAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_9) * L3;

    private boolean stuckInPosition;

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
        this.wristMotor = provider.getTalonSRX(ElectronicsConstants.ARM_WRIST_MOTOR_CAN_ID);

        this.shoulderMotor.setRelativeEncoder();
        // this.shoulderMotor.setInvertSensor(TuningConstants.ARM_SHOULDER_MOTOR_INVERT_SENSOR);
        this.shoulderMotor.setPositionConversionFactor(HardwareConstants.ARM_SHOULDER_TICK_DISTANCE);
        this.shoulderMotor.setVelocityConversionFactor(HardwareConstants.ARM_SHOULDER_TICK_DISTANCE);
        this.shoulderMotor.setInvertOutput(TuningConstants.ARM_SHOULDER_MOTOR_INVERT_OUTPUT);
        this.shoulderMotor.setPosition(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION);
        this.shoulderMotor.setNeutralMode(MotorNeutralMode.Brake);

        this.wristMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.wristMotor.setPosition(TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION * HardwareConstants.ARM_WRIST_TICKS_PER_DEGREE);
        this.wristMotor.setMotorOutputSettings(TuningConstants.ARM_WRIST_MOTOR_INVER_OUTPUT, MotorNeutralMode.Brake);
        this.wristMotor.setInvertSensor(TuningConstants.ARM_WRIST_MOTOR_INVERT_SENSOR);

        if (TuningConstants.ARM_USE_MM)
        {
            this.shoulderMotor.setPIDF(
                TuningConstants.ARM_SHOULDER_POSITIONAL_TMP_PID_KP,
                TuningConstants.ARM_SHOULDER_POSITIONAL_TMP_PID_KI,
                TuningConstants.ARM_SHOULDER_POSITIONAL_TMP_PID_KD,
                TuningConstants.ARM_SHOULDER_POSITIONAL_TMP_PID_KF,
                ArmMechanism.AltPidSlotId);

            this.wristMotor.setMotionMagicPIDF(
                TuningConstants.ARM_WRIST_POSITION_MM_PID_KP,
                TuningConstants.ARM_WRIST_POSITION_MM_PID_KI,
                TuningConstants.ARM_WRIST_POSITION_MM_PID_KD,
                TuningConstants.ARM_WRIST_POSITION_MM_PID_KF,
                TuningConstants.ARM_WRIST_POSITION_MM_CRUISE_VELOCITY,
                TuningConstants.ARM_WRIST_POSITION_MM_ACCELERATION,
                ArmMechanism.AltPidSlotId);
        }
        else
        {
            this.shoulderMotor.setPIDF(
                TuningConstants.ARM_SHOULDER_MOTOR_POSITIONAL_PID_KP,
                TuningConstants.ARM_SHOULDER_MOTOR_POSITIONAL_PID_KI,
                TuningConstants.ARM_SHOULDER_MOTOR_POSITIONAL_PID_KD,
                TuningConstants.ARM_SHOULDER_MOTOR_POSITIONAL_PID_KF,
                ArmMechanism.DefaultPidSlotId);

            this.wristMotor.setPIDF(
                TuningConstants.ARM_WRIST_MOTOR_PID_KP,
                TuningConstants.ARM_WRIST_MOTOR_PID_KI,
                TuningConstants.ARM_WRIST_MOTOR_PID_KD,
                TuningConstants.ARM_WRIST_MOTOR_PID_KF,
                ArmMechanism.DefaultPidSlotId);
        }

        this.shoulderPosition = TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION;
        this.wristPosition = TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION;

        this.desiredShoulderPosition = this.shoulderPosition;
        this.desiredWristPosition = this.wristPosition;

        if (TuningConstants.ARM_USE_MM)
        {
            this.shoulderTrapezoidMotionProfile = new TrapezoidProfile(
                TuningConstants.ARM_SHOULDER_TMP_PID_CRUISE_VELOC,
                TuningConstants.ARM_SHOULDER_TMP_PID_ACCEL);

            this.shoulderTMPCurrState = new TrapezoidProfile.State(this.shoulderPosition, 0.0);
            this.shoulderTMPGoalState = new TrapezoidProfile.State(this.shoulderPosition, 0.0);

            this.shoulderMotor.setSelectedSlot(ArmMechanism.AltPidSlotId);
            this.wristMotor.setSelectedSlot(ArmMechanism.AltPidSlotId);
        }
        else
        {
            this.shoulderTrapezoidMotionProfile = null;
            this.shoulderTMPCurrState = null;
            this.shoulderTMPGoalState = null;

            this.shoulderMotor.setSelectedSlot(ArmMechanism.DefaultPidSlotId);
            this.wristMotor.setSelectedSlot(ArmMechanism.DefaultPidSlotId);
        }

        if (this.inSimpleMode)
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            this.wristMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        }
        else if (TuningConstants.ARM_USE_MM)
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.Position);
            this.wristMotor.setControlMode(TalonSRXControlMode.MotionMagicPosition);
        }
        else
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.Position);
            this.wristMotor.setControlMode(TalonSRXControlMode.Position);
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

        if (TuningConstants.ARM_USE_GRAVITY_COMPENSATION)
        {
            this.interpolator =
                new BilinearInterpolator(
                    TuningConstants.ARM_GRAVITY_COMPENSATION_SHOULDER_SAMPLE_LOCATIONS,
                    TuningConstants.ARM_GRAVITY_COMPENSATION_WRIST_SAMPLE_LOCATIONS,
                    TuningConstants.ARM_GRAVITY_COMPENSATION_SAMPLES);
        }
        else
        {
            this.interpolator = null;
        }

        // setting initial IK variables
        this.updateIKVars(this.shoulderPosition, this.wristPosition);
    }

    @Override
    public void readSensors()
    {
        this.shoulderPosition = this.shoulderMotor.getPosition(); // in degrees (conversion to degrees included in setPositionConversionFactor)
        this.shoulderVelocity = this.shoulderMotor.getVelocity(); // in degrees/sec (conversion to degrees included in setVelocityConversionFactor)
        this.shoulderError = this.shoulderPosition - this.desiredShoulderPosition;
        this.wristPosition = this.wristMotor.getPosition() * HardwareConstants.ARM_WRIST_TICK_DISTANCE; // convert rotations to degrees
        this.wristVelocity = this.wristMotor.getVelocity() * HardwareConstants.ARM_WRIST_TICK_DISTANCE * 10.0; // convert ticks/100ms to degrees/sec
        this.wristError = this.wristMotor.getError();

        double shoulderCurrent = this.powerManager.getCurrent(ElectronicsConstants.ARM_SHOULDER_PDH_CHANNEL);
        double shoulderFollowerCurrent = this.powerManager.getCurrent(ElectronicsConstants.ARM_SHOULDER_FOLLOWER_PDH_CHANNEL);
        double wristCurrent = this.powerManager.getCurrent(ElectronicsConstants.ARM_WRIST_PDH_CHANNEL);
        double batteryVoltage = this.powerManager.getBatteryVoltage();

        this.shoulderPowerAverage = this.shoulderPowerAverageCalculator.update(((shoulderCurrent + shoulderFollowerCurrent) * 0.5) * batteryVoltage);
        this.wristPowerAverage = this.wristPowerAverageCalculator.update(wristCurrent * batteryVoltage);

        this.shoulderVelocityAverage = this.shoulderVelocityAverageCalculator.update(Math.abs(this.shoulderVelocity));
        this.wristVelocityAverage = this.wristVelocityAverageCalculator.update(Math.abs(this.wristVelocity));

        this.logger.logBoolean(LoggingKey.ArmExtensionBreaking, this.stuckInPosition);
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
        double elapsedTime = currTime - this.prevTime;
        boolean jumpAdjustment = false;

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
                this.wristMotor.setSelectedSlot(ArmMechanism.AltPidSlotId);
            }
            else
            {
                this.shoulderMotor.setSelectedSlot(ArmMechanism.DefaultPidSlotId);
                this.wristMotor.setSelectedSlot(ArmMechanism.DefaultPidSlotId);
            }

            this.desiredShoulderPosition = this.shoulderPosition;
            this.desiredWristPosition = this.wristPosition;

            this.shoulderSetpointChangedTime = currTime;
            this.wristSetpointChangedTime = currTime;

            this.shoulderStalled = false;
            this.wristStalled = false;
        }

        // ---------------> MAIN ARM CONTROL <------------------------

        if (this.driver.getDigital(DigitalOperation.ArmForceReset))
        {
            this.shoulderMotor.setPosition(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION);
            this.wristMotor.setPosition(TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);

            this.shoulderPosition = TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION; // Fully Retracted
            this.wristPosition = TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION;; // Fully Retracted

            this.desiredShoulderPosition = this.shoulderPosition;
            this.desiredWristPosition = this.wristPosition;

            this.shoulderSetpointChangedTime = currTime;
            this.wristSetpointChangedTime = currTime;

            this.shoulderStalled = false;
            this.wristStalled = false;
        }

        double shoulderPowerAdjustment = this.driver.getAnalog(AnalogOperation.ArmShoulderPower);
        double wristPowerAdjustment = this.driver.getAnalog(AnalogOperation.ArmWristPower);

        double shoulderPower = 0.0;
        double wristPower = 0.0;
        boolean useWristSimpleMode = false;
        boolean useShoulderSimpleMode = false;

        if (this.inSimpleMode)
        {
            useWristSimpleMode = true;
            useShoulderSimpleMode = true;

            shoulderPower = shoulderPowerAdjustment;
            wristPower = wristPowerAdjustment;

            this.shoulderSetpointChangedTime = currTime;
            this.wristSetpointChangedTime = currTime;

            this.shoulderStalled = false;
            this.wristStalled = false;
        }
        else
        {
            if (shoulderPowerAdjustment != 0.0)
            {
                useShoulderSimpleMode = true;

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
                useWristSimpleMode = true;

                // Updated desired to current position
                this.desiredWristPosition = this.wristPosition;

                // Update changed time and stalled constants
                this.wristSetpointChangedTime = currTime;
                this.wristStalled = false;

                // Update this power value
                wristPower = wristPowerAdjustment;
            }
            else
            {
                double newDesiredShoulderPosition = this.driver.getAnalog(AnalogOperation.ArmShoulderPositionSetpoint);
                double newDesiredWristPosition = this.driver.getAnalog(AnalogOperation.ArmWristPositionSetpoint);
                if (newDesiredWristPosition == TuningConstants.MAGIC_NULL_VALUE)
                {
                    double newDesiredAbsoluteWristAngle = this.driver.getAnalog(AnalogOperation.ArmAbsWristAngle);
                    if (newDesiredAbsoluteWristAngle != TuningConstants.MAGIC_NULL_VALUE)
                    {
                        newDesiredWristPosition = this.switchToTheta2(newDesiredAbsoluteWristAngle);
                    }
                }

                double newShoulderPositionAdjustment = this.driver.getAnalog(AnalogOperation.ArmShoulderAdjustment) * TuningConstants.ARM_SHOULDER_PID_ADJUST_VEL * elapsedTime;
                double newWristPositionAdjustment = this.driver.getAnalog(AnalogOperation.ArmWristAdjustment) * TuningConstants.ARM_WRIST_PID_ADJUST_VEL * elapsedTime;

                this.logger.logNumber(LoggingKey.ArmShoulderPosAdjustment, newShoulderPositionAdjustment);
                this.logger.logNumber(LoggingKey.ArmWristPosAdjustment, newWristPositionAdjustment);

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
                        jumpAdjustment = true;
                    }
                }

                this.desiredShoulderPosition += newShoulderPositionAdjustment;
                this.desiredWristPosition += newWristPositionAdjustment;
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

        double currentDesiredShoulderPosition = this.desiredShoulderPosition;
        double currentDesiredWristPosition = this.desiredWristPosition;
        if (TuningConstants.ARM_USE_MM)
        {
            TrapezoidProfile.State curr = this.shoulderTMPCurrState;
            TrapezoidProfile.State goal = this.shoulderTMPGoalState;

            if (goal.updatePosition(desiredShoulderPosition) && jumpAdjustment)
            {
                curr.updatePosition(this.shoulderPosition);
            }

            if (this.shoulderTrapezoidMotionProfile.update(elapsedTime, curr, goal))
            {
                currentDesiredShoulderPosition = curr.getPosition();
            }
        }

        double[] angles = this.limitedAngles(currentDesiredShoulderPosition, this.desiredWristPosition);
        if (TuningConstants.USE_IK_CONSTRAINTS)
        {
            currentDesiredShoulderPosition = angles[0];
            currentDesiredWristPosition = angles[1];
            if (!TuningConstants.ARM_USE_MM)
            {
                this.desiredShoulderPosition = currentDesiredShoulderPosition;
                this.desiredWristPosition = currentDesiredWristPosition;
            }
        }

        this.updateIKVars(currentDesiredShoulderPosition, currentDesiredWristPosition);

        double feedForward = 0.0;
        if (TuningConstants.ARM_USE_GRAVITY_COMPENSATION)
        {
            feedForward = this.interpolator.sample(currentDesiredShoulderPosition, currentDesiredWristPosition);
        }

        if (!useShoulderSimpleMode)
        {
            if (this.shoulderStalled)
            {
                this.shoulderMotor.stop();
            }
            else
            {
                this.shoulderMotor.set(
                    SparkMaxControlMode.Position,
                    currentDesiredShoulderPosition,
                    feedForward);
            }
        }
        else
        {
            this.shoulderMotor.set(SparkMaxControlMode.PercentOutput, shoulderPower);
        }

        if (!useWristSimpleMode)
        {
            if (this.wristStalled)
            {
                this.wristMotor.stop();
            }
            else
            {
                this.wristMotor.set(
                    TuningConstants.ARM_USE_MM ? TalonSRXControlMode.MotionMagicPosition : TalonSRXControlMode.Position,
                    currentDesiredWristPosition * HardwareConstants.ARM_WRIST_TICKS_PER_DEGREE,
                    0.0);
            }
        }
        else
        {
            this.wristMotor.set(TalonSRXControlMode.PercentOutput, wristPower);
        }

        this.logger.logNumber(LoggingKey.ArmShoulderOutput, this.shoulderMotor.getOutput());
        this.logger.logNumber(LoggingKey.ArmWristOutput, this.wristMotor.getOutput());
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
        this.wristMotor.stop();
        this.prevTime = 0.0;
    }

    private void updateIKVars(double shoulderAngle, double wristAngle)
    {
        this.theta_1 = shoulderAngle;
        this.theta_2 = wristAngle;
        this.theta_3 = 180.0 - this.theta_1;
        this.theta_4 = 360 - this.theta_3 - this.theta_2;
        this.theta_5 = this.theta_4 - 170.0;
        this.theta_8 = this.theta_4 - this.theta_6;
        this.theta_9 = this.theta_5 + this.theta_7;

        this.shoulderJointAbsPosX = HardwareConstants.ARM_TO_CENTER_ROBOT_X_OFFSET;
        this.shoulderJointAbsPosZ = HardwareConstants.ARM_TO_CENTER_ROBOT_Z_OFFSET;
        this.wristAbsPosX = this.shoulderJointAbsPosX + Helpers.cosd(this.theta_1) * L1;
        this.wristAbsPosZ = this.shoulderJointAbsPosZ + Helpers.sind(this.theta_1) * L1;
        this.shooterBottomAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_4) * L2x;
        this.shooterBottomAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_4) * L2x;
        this.shooterTopAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_8) * L2;
        this.shooterTopAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_8) * L2;
        this.intakeBottomAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_5) * L3x;
        this.intakeBottomAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_5) * L3x;
        this.intakeTopAbsPosX = this.wristAbsPosX + Helpers.cosd(this.theta_9) * L3;
        this.intakeTopAbsPosZ = this.wristAbsPosZ + Helpers.sind(this.theta_9) * L3;
    }

    private double[] limitedAngles(double desShoulder, double desWrist)
    {
        this.updateIKVars(desShoulder, desWrist);
        double[] positions = new double[2];
        positions[0] = desShoulder;
        positions[1] = desWrist;

        boolean extensionTop =
            this.intakeTopAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT ||
            this.shooterTopAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT;
        boolean extensionFront =
            this.intakeTopAbsPosX > HardwareConstants.MAX_ROBOT_EXTENSION + HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0 ||
            this.intakeBottomAbsPosX > HardwareConstants.MAX_ROBOT_EXTENSION + HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0;
        boolean extensionBack =
            -this.intakeBottomAbsPosX > HardwareConstants.MAX_ROBOT_EXTENSION + HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0 ||
            -this.intakeTopAbsPosX > HardwareConstants.MAX_ROBOT_EXTENSION + HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0;

        // Instant limiting
        if (extensionBack)
        {
            positions[0] = this.shoulderPosition;
            positions[1] = this.wristPosition;
        }
        else if (this.intakeBottomAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT || this.shooterBottomAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT)
        {
            positions[0] = this.shoulderPosition;
            positions[1] = this.wristPosition;
        }
        // hiting robot
        else if ( (this.intakeTopAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.intakeTopAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION)
            || (this.intakeBottomAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.intakeBottomAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION)
            || (this.shooterTopAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.shooterTopAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION)
            || (this.shooterBottomAbsPosZ < HardwareConstants.MIN_USABLE_HEIGHT && Math.abs(this.shooterBottomAbsPosX) < HardwareConstants.ROBOT_FRAME_DIMENSION))
        {
            positions[0] = this.shoulderPosition;
            positions[1] = this.wristPosition;
        }
        // hitting ground
        else if ( (this.intakeTopAbsPosZ < 0) || (this.intakeBottomAbsPosZ < 0))
        {
            positions[0] = this.shoulderPosition;
            positions[0] = this.wristPosition;
        }
        // continous limiting top
        else if (extensionTop)
        {
            boolean intakeSide = intakeTopAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT;
            boolean shooterSide = shooterTopAbsPosZ > HardwareConstants.MAX_ROBOT_HEIGHT;
            double desiredDistance = HardwareConstants.MAX_ROBOT_HEIGHT - this.wristAbsPosZ;

            if (intakeSide && shooterSide)
            {
                positions[0] = this.shoulderPosition;
                positions[1] = this.wristPosition;
            }
            else if (intakeSide && intakeTopAbsPosX > wristAbsPosX)
            {
                double temp_theta_9 = -Helpers.asind(desiredDistance / L3); // meant to return a positive value, negative added since asin(0.5) is negative
                double temp_theta_4 = temp_theta_9 - this.theta_7 + 180.0;
                double temp_wrist_angle = 180 + this.theta_1 - temp_theta_4;
                positions[0] = desShoulder;
                positions[1] = temp_wrist_angle;
            }
            else if (shooterSide && shooterTopAbsPosX > wristAbsPosX)
            {
                double temp_theta_8 = -Helpers.asind(desiredDistance / L2); // meant to return a positive value, negative added since asin(0.5) is negative
                double temp_theta_4 = temp_theta_8 + this.theta_6;
                double temp_wrist_angle = 180 + this.theta_1 - temp_theta_4;
                positions[0] = desShoulder;
                positions[1] = temp_wrist_angle;
            }
            else
            {
                positions[0] = this.shoulderPosition;
                positions[1] = this.wristPosition;
            }
        }
        // continous limiting front
        else if (extensionFront)
        {
            boolean intakeTop = intakeTopAbsPosX > HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0 + HardwareConstants.MAX_ROBOT_EXTENSION;
            boolean intakeBottom = intakeBottomAbsPosX > HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0 + HardwareConstants.MAX_ROBOT_EXTENSION;
            double desiredDistance = HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0 + HardwareConstants.MAX_ROBOT_EXTENSION - this.wristAbsPosX;

            if (intakeTop && intakeBottom)
            {
                positions[0] = this.shoulderPosition;
                positions[1] = this.wristPosition;
            }
            else if (intakeTop && intakeTopAbsPosZ < wristAbsPosZ)
            {
                double temp_theta_9 = -Helpers.acosd(desiredDistance / L3); // meant to return a negative value, negative added since acos(0.5) is positive
                double temp_theta_4 = temp_theta_9 - this.theta_7 + 180.0;
                double temp_wrist_angle = 180 + this.theta_1 - temp_theta_4;
                positions[0] = desShoulder;
                positions[1] = temp_wrist_angle;
            }
            else if (intakeBottom)
            {
                double temp_theta_5 = Helpers.acosd(desiredDistance / L3x);
                double temp_theta_4 = temp_theta_5 + 180.0;
                double temp_wrist_angle = 180 + this.theta_1 - temp_theta_4;
                positions[0] = desShoulder;
                positions[1] = temp_wrist_angle;
            }
            else
            {
                positions[0] = this.shoulderPosition;
                positions[1] = this.wristPosition;

            }
        }

        if (positions[0] == this.shoulderPosition && positions[1] == this.wristPosition)
        {
            this.stuckInPosition = true;
        }
        else
        {
            this.stuckInPosition = false;
        }

        return positions;
    }

    public double getTheta1()
    {
        return this.theta_1;
    }

    public double getTheta2()
    {
        return this.theta_2;
    }

    public double getAbsoluteAngleOfShot()
    {
        return this.theta_4;
    }

    private double switchToTheta2(double desiredAbsWrist)
    {
        return 180 + this.theta_1 - desiredAbsWrist;
    }

    public double[] wristJointAbsPosition()
    {
        double[] absWristPosition = new double[2];
        absWristPosition[0] = this.wristAbsPosX;
        absWristPosition[1] = this.wristAbsPosZ;

        return absWristPosition;
    }

    public boolean getStuckInPosition()
    {
        return this.stuckInPosition;
    }
}
