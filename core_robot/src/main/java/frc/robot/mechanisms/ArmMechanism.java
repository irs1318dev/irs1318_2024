package frc.robot.mechanisms;

import frc.lib.controllers.TrapezoidProfile;
import frc.lib.driver.IDriver;
import frc.lib.filters.FloatingAverageCalculator;
import frc.lib.helpers.BilinearInterpolator;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.lib.helpers.Pair;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.*;
import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.SmartDashboardSelectionManager;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ArmMechanism implements IMechanism
{
    private enum JumpProtectionReason
    {
        None,
        Startup,
        Stall,
        Reset,
        PositionChange,
        IK,
    }

    private static final int DefaultPidSlotId = 0;
    private static final int AltPidSlotId = 1;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;
    private final PowerManager powerManager;
    private final SmartDashboardSelectionManager selectionManager;

    private double prevTime;

    private final ISparkMax shoulderMotor;
    private final ISparkMax wristMotor;
    private final IDutyCycleEncoder wristAbsoluteEncoder;
    private final IDutyCycleEncoder shoulderAbsoluteEncoder;

    private double desiredShoulderPosition;
    private double desiredWristPosition;

    private double currentDesiredShoulderPosition;
    private double currentDesiredWristPosition;

    private double shoulderPosition;
    private double shoulderVelocity;

    private double wristPosition;
    private double wristVelocity;

    private Double wristAbsoluteEncoderPosition;
    private Double shoulderAbsoluteEncoderPosition;

    private double shoulderError;
    private double wristError;

    private double shoulderSetpointChangedTime;
    private double wristSetpointChangedTime;
    private boolean shoulderStalled;
    private boolean wristStalled;
    
    private boolean wristLimitSwitchHit;

    private FloatingAverageCalculator shoulderMasterPowerAverageCalculator;
    private FloatingAverageCalculator shoulderFollowerPowerAverageCalculator;
    private FloatingAverageCalculator wristPowerAverageCalculator;

    private double shoulderPowerAverage;
    private double shoulderMasterPowerAverage;
    private double shoulderFollowerPowerAverage;
    private double wristPowerAverage;

    private FloatingAverageCalculator shoulderVelocityAverageCalculator;
    private FloatingAverageCalculator wristVelocityAverageCalculator;

    private double shoulderVelocityAverage;
    private double wristVelocityAverage;

    private final TrapezoidProfile shoulderTrapezoidMotionProfile;
    private final TrapezoidProfile.State shoulderTMPCurrState;
    private final TrapezoidProfile.State shoulderTMPGoalState;

    private final TrapezoidProfile wristTrapezoidMotionProfile;
    private final TrapezoidProfile.State wristTMPCurrState;
    private final TrapezoidProfile.State wristTMPGoalState;

    // Gravity Compensation
    private final BilinearInterpolator interpolator;

    private final ArmKinematicsCalculator armKinematicsCalculator;
    private final Pair<Double, Double> kinematicsLimitedAngles;

    private double lastLegalWristPosition;
    private double lastLegalShoulderPosition;

    private boolean wasEnabled;
    private boolean inSimpleMode;
    private JumpProtectionReason updateCurrWristPosition;
    private JumpProtectionReason updateCurrShoulderPosition;

    private boolean useThroughBoreEncoders;

    @Inject
    public ArmMechanism(
        IRobotProvider provider,
        IDriver driver,
        LoggingManager logger,
        ITimer timer,
        PowerManager powerManager,
        SmartDashboardSelectionManager selectionManager)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;
        this.powerManager = powerManager;
        this.selectionManager = selectionManager;

        this.inSimpleMode = TuningConstants.ARM_USE_SIMPLE_MODE;
        this.updateCurrShoulderPosition = JumpProtectionReason.Startup;
        this.updateCurrWristPosition = JumpProtectionReason.Startup;

        this.shoulderMotor = provider.getSparkMax(ElectronicsConstants.ARM_SHOULDER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.wristMotor = provider.getSparkMax(ElectronicsConstants.ARM_WRIST_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);

        this.shoulderMotor.setRelativeEncoder();
        this.shoulderMotor.setPositionConversionFactor(HardwareConstants.ARM_SHOULDER_TICK_DISTANCE);
        this.shoulderMotor.setVelocityConversionFactor(HardwareConstants.ARM_SHOULDER_TICK_VELOCITY);
        this.shoulderMotor.setInvertOutput(TuningConstants.ARM_SHOULDER_MOTOR_INVERT_OUTPUT);
        this.shoulderMotor.setPosition(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION);
        this.shoulderMotor.setNeutralMode(MotorNeutralMode.Brake);

        this.wristMotor.setRelativeEncoder();
        this.wristMotor.setInvertOutput(TuningConstants.ARM_WRIST_MOTOR_INVERT_OUTPUT);
        this.wristMotor.setReverseLimitSwitch(ElectronicsConstants.ARM_WRIST_LIMIT_SWITCH_ENABLED, ElectronicsConstants.ARM_WRIST_LIMIT_SWITCH_NORMALLY_OPEN);
        this.wristMotor.setPositionConversionFactor(HardwareConstants.ARM_WRIST_TICK_DISTANCE);
        this.wristMotor.setVelocityConversionFactor(HardwareConstants.ARM_WRIST_TICK_VELOCITY);
        this.wristMotor.setPosition(TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);
        this.wristMotor.setNeutralMode(MotorNeutralMode.Coast);

        this.wristAbsoluteEncoder = provider.getDutyCycleEncoder(ElectronicsConstants.ARM_WRIST_ABSOLUTE_ENCODER_DIO_CHANNEL);
        this.wristAbsoluteEncoder.setDutyCycleRange(ElectronicsConstants.REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MIN, ElectronicsConstants.REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MAX);
        this.wristAbsoluteEncoder.setDistancePerRotation(-HardwareConstants.ARM_WRIST_ABSOLUTE_ENCODER_TICK_DISTANCE);
        this.wristAbsoluteEncoder.setPositionOffset(TuningConstants.ARM_WRIST_ABSOLUTE_ENCODER_OFFSET);

        this.shoulderAbsoluteEncoder = provider.getDutyCycleEncoder(ElectronicsConstants.ARM_SHOULDER_ABSOLUTE_ENCODER_DIO_CHANNEL);
        this.shoulderAbsoluteEncoder.setDutyCycleRange(ElectronicsConstants.REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MIN, ElectronicsConstants.REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MAX);
        this.shoulderAbsoluteEncoder.setDistancePerRotation(HardwareConstants.ARM_SHOULDER_ABSOLUTE_ENCODER_TICK_DISTANCE);
        this.shoulderAbsoluteEncoder.setPositionOffset(TuningConstants.ARM_SHOULDER_ABSOLUTE_ENCODER_OFFSET);

        if (TuningConstants.ARM_USE_MM)
        {
            this.shoulderMotor.setPIDF(
                TuningConstants.ARM_SHOULDER_MOTOR_POSITIONAL_TMP_PID_KP,
                TuningConstants.ARM_SHOULDER_MOTOR_POSITIONAL_TMP_PID_KI,
                TuningConstants.ARM_SHOULDER_MOTOR_POSITIONAL_TMP_PID_KD,
                TuningConstants.ARM_SHOULDER_MOTOR_POSITIONAL_TMP_PID_KF,
                ArmMechanism.AltPidSlotId);

            this.wristMotor.setPIDF(
                TuningConstants.ARM_WRIST_MOTOR_POSITIONAL_TMP_PID_KP,
                TuningConstants.ARM_WRIST_MOTOR_POSITIONAL_TMP_PID_KI,
                TuningConstants.ARM_WRIST_MOTOR_POSITIONAL_TMP_PID_KD,
                TuningConstants.ARM_WRIST_MOTOR_POSITIONAL_TMP_PID_KF,
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
                TuningConstants.ARM_WRIST_MOTOR_POSITIONAL_PID_KP,
                TuningConstants.ARM_WRIST_MOTOR_POSITIONAL_PID_KI,
                TuningConstants.ARM_WRIST_MOTOR_POSITIONAL_PID_KD,
                TuningConstants.ARM_WRIST_MOTOR_POSITIONAL_PID_KF,
                ArmMechanism.DefaultPidSlotId);
        }

        this.shoulderPosition = TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION;
        this.wristPosition = TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION;

        this.desiredShoulderPosition = this.shoulderPosition;
        this.desiredWristPosition = this.wristPosition;

        if (TuningConstants.ARM_USE_MM)
        {
            this.shoulderTrapezoidMotionProfile = new TrapezoidProfile(
                TuningConstants.ARM_SHOULDER_MOTOR_TMP_PID_CRUISE_VELOC,
                TuningConstants.ARM_SHOULDER_MOTOR_TMP_PID_ACCEL);
            this.shoulderTMPCurrState = new TrapezoidProfile.State(this.shoulderPosition, 0.0);
            this.shoulderTMPGoalState = new TrapezoidProfile.State(this.shoulderPosition, 0.0);

            this.shoulderMotor.setSelectedSlot(ArmMechanism.AltPidSlotId);

            this.wristTrapezoidMotionProfile = new TrapezoidProfile(
                TuningConstants.ARM_WRIST_MOTOR_TMP_PID_CRUISE_VELOC,
                TuningConstants.ARM_WRIST_MOTOR_TMP_PID_ACCEL);
            this.wristTMPCurrState = new TrapezoidProfile.State(this.wristPosition, 0.0);
            this.wristTMPGoalState = new TrapezoidProfile.State(this.wristPosition, 0.0);

            this.wristMotor.setSelectedSlot(ArmMechanism.AltPidSlotId);
        }
        else
        {
            this.shoulderTrapezoidMotionProfile = null;
            this.shoulderTMPCurrState = null;
            this.shoulderTMPGoalState = null;

            this.shoulderMotor.setSelectedSlot(ArmMechanism.DefaultPidSlotId);

            this.wristTrapezoidMotionProfile = null;
            this.wristTMPCurrState = null;
            this.wristTMPGoalState = null;
            this.wristMotor.setSelectedSlot(ArmMechanism.DefaultPidSlotId);
        }

        if (this.inSimpleMode)
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            this.wristMotor.setControlMode(SparkMaxControlMode.PercentOutput);
        }
        else if (TuningConstants.ARM_USE_MM)
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.Position);
            this.wristMotor.setControlMode(SparkMaxControlMode.Position);
        }
        else
        {
            this.shoulderMotor.setControlMode(SparkMaxControlMode.Position);
            this.wristMotor.setControlMode(SparkMaxControlMode.Position);
        }

        this.shoulderMotor.burnFlash();
        this.wristMotor.burnFlash();

        ISparkMax shoulderFollowerMotor = provider.getSparkMax(ElectronicsConstants.ARM_SHOULDER_FOLLOWER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        shoulderFollowerMotor.setInvertOutput(TuningConstants.ARM_SHOULDER_MOTOR_FOLLOWER_INVERT_OUTPUT);
        shoulderFollowerMotor.setNeutralMode(MotorNeutralMode.Brake);
        shoulderFollowerMotor.follow(this.shoulderMotor);
        shoulderFollowerMotor.burnFlash();

        this.shoulderMasterPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_SHOULDER_POWER_TRACKING_DURATION, TuningConstants.ARM_SHOULDER_POWER_SAMPLES_PER_SECOND);
        this.shoulderFollowerPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ARM_SHOULDER_POWER_TRACKING_DURATION, TuningConstants.ARM_SHOULDER_POWER_SAMPLES_PER_SECOND);
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
        this.armKinematicsCalculator = new ArmKinematicsCalculator(this.shoulderPosition, this.wristPosition);
        this.kinematicsLimitedAngles = new Pair<Double, Double>(this.shoulderPosition, this.wristPosition);

        this.lastLegalShoulderPosition = TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION;
        this.lastLegalWristPosition = TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION;

        this.useThroughBoreEncoders = TuningConstants.ARM_USE_WRIST_ABSOLUTE_ENCODER_RESET || TuningConstants.ARM_USE_SHOULDER_ABSOLUTE_ENCODER_RESET;
        this.wasEnabled = false;
    }

    @Override
    public void readSensors()
    {
        this.shoulderPosition = this.shoulderMotor.getPosition(); // in degrees (conversion to degrees included in setPositionConversionFactor)
        this.shoulderVelocity = this.shoulderMotor.getVelocity(); // in degrees/sec (conversion to degrees included in setVelocityConversionFactor)
        this.shoulderError = this.shoulderPosition - this.currentDesiredShoulderPosition;
        this.wristPosition = this.wristMotor.getPosition(); // convert rotations to degrees
        this.wristVelocity = this.wristMotor.getVelocity(); // convert ticks/100ms to degrees/sec
        this.wristError = this.wristPosition - this.currentDesiredWristPosition;

        this.wristAbsoluteEncoderPosition =
            !this.wristAbsoluteEncoder.isConnected() ?
                null : Helpers.updateAngleRange180(this.wristAbsoluteEncoder.getDistance() - 5.0);

        this.shoulderAbsoluteEncoderPosition =
            !this.shoulderAbsoluteEncoder.isConnected() ?
                null : Helpers.updateAngleRange180(this.shoulderAbsoluteEncoder.getDistance() - 39.5);

        // System.out.println(
        //     String.format(
        //         "Connected: %b, absPos: %f, freq %d, dist %f",
        //         this.wristAbsoluteEncoder.isConnected(),
        //         this.wristAbsoluteEncoder.getAbsolutePosition(),
        //         this.wristAbsoluteEncoder.getFrequency(),
        //         this.wristAbsoluteEncoder.getDistance()));

        this.wristLimitSwitchHit = this.wristMotor.getReverseLimitSwitchStatus();

        double shoulderCurrent = this.powerManager.getCurrent(ElectronicsConstants.ARM_SHOULDER_PDH_CHANNEL);
        double shoulderFollowerCurrent = this.powerManager.getCurrent(ElectronicsConstants.ARM_SHOULDER_FOLLOWER_PDH_CHANNEL);
        double wristCurrent = this.powerManager.getCurrent(ElectronicsConstants.ARM_WRIST_PDH_CHANNEL);
        double batteryVoltage = this.powerManager.getBatteryVoltage();

        this.shoulderMasterPowerAverage = this.shoulderMasterPowerAverageCalculator.update(shoulderCurrent * batteryVoltage);
        this.shoulderFollowerPowerAverage = this.shoulderFollowerPowerAverageCalculator.update(shoulderFollowerCurrent * batteryVoltage);
        this.shoulderPowerAverage = 0.5 * (this.shoulderMasterPowerAverage + this.shoulderFollowerPowerAverage);
        this.wristPowerAverage = this.wristPowerAverageCalculator.update(wristCurrent * batteryVoltage);

        this.shoulderVelocityAverage = this.shoulderVelocityAverageCalculator.update(Math.abs(this.shoulderVelocity));
        this.wristVelocityAverage = this.wristVelocityAverageCalculator.update(Math.abs(this.wristVelocity));

        // Check for power discrepancy between the master and follower motors for the shoulder
        boolean isShoulderMotorPowerDiscrepancy = false;
        if (this.shoulderPowerAverage > TuningConstants.ARM_SHOULDER_MOTOR_POWER_MIN_DIFFERENCE)
        {
            isShoulderMotorPowerDiscrepancy = Math.abs(this.shoulderMasterPowerAverage - this.shoulderFollowerPowerAverage) / this.shoulderPowerAverage >= TuningConstants.ARM_SHOULDER_MOTOR_POWER_DIFFERENCE;
        }

        this.logger.logNumber(LoggingKey.ArmShoulderPosition, this.shoulderPosition);
        this.logger.logNumber(LoggingKey.ArmShoulderVelocity, this.shoulderVelocity);
        this.logger.logNumber(LoggingKey.ArmShoulderVelocityAverage, this.shoulderVelocityAverage);
        this.logger.logNumber(LoggingKey.ArmShoulderError, this.shoulderError);
        this.logger.logBoolean(LoggingKey.ArmShoulderMotorPowerDiscrepancy, isShoulderMotorPowerDiscrepancy);
        this.logger.logNumber(LoggingKey.ArmShoulderPowerAverage, this.shoulderPowerAverage);
        this.logger.logNumber(LoggingKey.ArmWristPosition, this.wristPosition);
        this.logger.logNumber(LoggingKey.ArmWristVelocity, this.wristVelocity);
        this.logger.logNumber(LoggingKey.ArmWristVelocityAverage, this.wristVelocityAverage);
        this.logger.logNumber(LoggingKey.ArmWristError, this.wristError);
        this.logger.logNumber(LoggingKey.ArmWristPowerAverage, this.wristPowerAverage);
        this.logger.logBoolean(LoggingKey.ArmWristLimitSwitch, this.wristLimitSwitchHit);
        this.logger.logNumber(LoggingKey.ArmWristAbsoluteEncoderPosition, this.wristAbsoluteEncoderPosition);
        this.logger.logNumber(LoggingKey.ArmShoulderAbsoluteEncoderPosition, this.shoulderAbsoluteEncoderPosition);
    }

    @Override
    public void update(RobotMode mode)
    {
        double currTime = this.timer.get();
        double elapsedTime = currTime - this.prevTime;
        ExceptionHelpers.Assert(elapsedTime < 0.5, "ElapsedTime too long! %.4f", elapsedTime);

        double wristSlopAdjustment = 0.0;
        if (this.selectionManager.getUseWristSlop())
        {
            wristSlopAdjustment = this.selectionManager.getWristSlopAdjustment() * TuningConstants.ARM_SLOP_ADJUSTMENT_MULTIPLIER;
        }

        if (this.driver.getDigital(DigitalOperation.ArmEnableThroughBore))
        {
            this.useThroughBoreEncoders = true;
        }
        else if (this.driver.getDigital(DigitalOperation.ArmDisableThroughBore))
        {
            this.useThroughBoreEncoders = false;
        }

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
            this.shoulderMotor.burnFlash();
            if (!this.wasEnabled)
            {
                if (TuningConstants.ARM_USE_COAST_ON_DISABLE)
                {
                    this.wristMotor.setNeutralMode(MotorNeutralMode.Brake);
                }

                this.prevTime = currTime;
                elapsedTime = 0.01;
                this.wasEnabled = true;
            }

            this.wristMotor.setPosition(TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);
            this.wristMotor.burnFlash();

            this.shoulderPosition = TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION; // Fully Retracted
            this.wristPosition = TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION;; // Fully Retracted

            this.desiredShoulderPosition = this.shoulderPosition;
            this.desiredWristPosition = this.wristPosition;

            this.shoulderSetpointChangedTime = currTime;
            this.wristSetpointChangedTime = currTime;

            this.shoulderStalled = false;
            this.wristStalled = false;

            this.updateCurrShoulderPosition = JumpProtectionReason.Startup;
            this.updateCurrWristPosition = JumpProtectionReason.Startup;
        }
        else if (!this.wasEnabled)
        {
            // note - we want to avoid double-burnFlash, so anything we do here we should do within the arm force reset above
            if (TuningConstants.ARM_USE_COAST_ON_DISABLE)
            {
                this.wristMotor.setNeutralMode(MotorNeutralMode.Brake);
                this.wristMotor.burnFlash();
            }

            this.prevTime = currTime;
            elapsedTime = 0.01;
            this.wasEnabled = true;
        }

        boolean armStop = this.driver.getDigital(DigitalOperation.ArmStop);

        double shoulderPowerAdjustment = this.driver.getAnalog(AnalogOperation.ArmShoulderPower);
        double wristPowerAdjustment = this.driver.getAnalog(AnalogOperation.ArmWristPower);

        double shoulderPower = 0.0;
        double wristPower = 0.0;
        boolean useShoulderSimpleMode = false;
        boolean useWristSimpleMode = false;

        if (this.inSimpleMode)
        {
            useShoulderSimpleMode = true;
            useWristSimpleMode = true;

            shoulderPower = shoulderPowerAdjustment;
            wristPower = wristPowerAdjustment;

            this.shoulderSetpointChangedTime = currTime;
            this.wristSetpointChangedTime = currTime;

            this.shoulderStalled = false;
            this.wristStalled = false;
        }
        else if (armStop)
        {
            useShoulderSimpleMode = true;
            useWristSimpleMode = true;

            //  -------------------- > should be using power adjustments <------------------------------ 
            shoulderPower = 0.0;
            wristPower = 0.0;
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
                this.updateCurrShoulderPosition = JumpProtectionReason.PositionChange;

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
                this.updateCurrWristPosition = JumpProtectionReason.PositionChange;

                // Update this power value
                wristPower = wristPowerAdjustment;
            }

            if (wristPowerAdjustment == 0.0 && shoulderPowerAdjustment == 0.0)
            {
                // preset values and resetting
                double newDesiredShoulderPosition = this.driver.getAnalog(AnalogOperation.ArmShoulderPositionSetpoint);
                double newDesiredWristPosition = this.driver.getAnalog(AnalogOperation.ArmWristPositionSetpoint);

                // shooter calculates for abs shot angle not wrist angle
                if (newDesiredWristPosition == TuningConstants.MAGIC_NULL_VALUE)
                {
                    double newDesiredAbsoluteWristAngle = this.driver.getAnalog(AnalogOperation.ArmAbsWristAngle);
                    if (newDesiredAbsoluteWristAngle != TuningConstants.MAGIC_NULL_VALUE)
                    {
                        // System.out.println(newDesiredAbsoluteWristAngle);
                        double tempVal = this.armKinematicsCalculator.switchToTheta2(newDesiredAbsoluteWristAngle);
                        this.logger.logNumber(LoggingKey.ActWristDesired, tempVal);
                        newDesiredWristPosition = tempVal;
                    }
                }

                double newShoulderPositionAdjustment = 0.0;
                double newWristPositionAdjustment = 0.0;

                if (newDesiredShoulderPosition != TuningConstants.MAGIC_NULL_VALUE ||
                    newDesiredWristPosition != TuningConstants.MAGIC_NULL_VALUE)
                {
                    // controlled by macro
                    if (newDesiredShoulderPosition != TuningConstants.MAGIC_NULL_VALUE &&
                        (!Helpers.RoughEquals(this.desiredShoulderPosition, newDesiredShoulderPosition, 0.1) ||
                         (!Helpers.RoughEquals(this.shoulderPosition, newDesiredShoulderPosition, 1.0) && this.shoulderStalled)))
                    {
                        if (this.updateCurrShoulderPosition != JumpProtectionReason.None &&
                            !Helpers.RoughEquals(this.desiredShoulderPosition, newDesiredShoulderPosition, 0.1))
                        {
                            this.updateCurrShoulderPosition = JumpProtectionReason.PositionChange;
                        }

                        this.shoulderSetpointChangedTime = currTime;
                        this.shoulderStalled = false;

                        this.desiredShoulderPosition = newDesiredShoulderPosition;
                    }

                    if (newDesiredWristPosition != TuningConstants.MAGIC_NULL_VALUE &&
                        (!Helpers.RoughEquals(this.desiredWristPosition, newDesiredWristPosition, 0.1) ||
                         (!Helpers.RoughEquals(this.wristPosition, newDesiredWristPosition, 1.0) && this.wristStalled)))
                    {
                        if (this.updateCurrWristPosition != JumpProtectionReason.None &&
                            !Helpers.RoughEquals(this.desiredWristPosition, newDesiredWristPosition, 0.1))
                        {
                            this.updateCurrWristPosition = JumpProtectionReason.PositionChange;
                        }

                        this.wristSetpointChangedTime = currTime;
                        this.wristStalled = false;

                        this.desiredWristPosition = newDesiredWristPosition;
                    }
                }
                else
                {
                    // position adjustemnt with co-driver analog axis
                    newShoulderPositionAdjustment = this.driver.getAnalog(AnalogOperation.ArmShoulderAdjustment) * TuningConstants.ARM_SHOULDER_PID_ADJUST_VEL * elapsedTime;
                    newWristPositionAdjustment = this.driver.getAnalog(AnalogOperation.ArmWristAdjustment) * TuningConstants.ARM_WRIST_PID_ADJUST_VEL * elapsedTime;

                    if (newShoulderPositionAdjustment != 0.0)
                    {
                        this.shoulderSetpointChangedTime = currTime;
                        this.shoulderStalled = false;
                        this.desiredShoulderPosition += newShoulderPositionAdjustment;
                    }

                    if (newWristPositionAdjustment != 0.0)
                    {
                        this.wristSetpointChangedTime = currTime;
                        this.wristStalled = false;
                        this.desiredWristPosition += newWristPositionAdjustment;
                    }
                }

                this.logger.logNumber(LoggingKey.ArmShoulderPosAdjustment, newShoulderPositionAdjustment);
                this.logger.logNumber(LoggingKey.ArmWristPosAdjustment, newWristPositionAdjustment);

                if (newDesiredShoulderPosition != TuningConstants.MAGIC_NULL_VALUE ||
                    newDesiredWristPosition != TuningConstants.MAGIC_NULL_VALUE ||
                    newShoulderPositionAdjustment != 0.0 ||
                    newWristPositionAdjustment != 0.0)
                {
                    // clamp the values to the allowed ranges if we are making any change to position using position-based movement
                    double clampedDesiredShoulderPosition = Helpers.EnforceRange(this.desiredShoulderPosition, TuningConstants.ARM_SHOULDER_MIN_POSITION, TuningConstants.ARM_SHOULDER_MAX_POSITION);
                    double clampedDesiredWristPosition = Helpers.EnforceRange(this.desiredWristPosition, TuningConstants.ARM_WRIST_MIN_POSITION, TuningConstants.ARM_WRIST_MAX_POSITION);
                    this.logger.logBoolean(LoggingKey.ArmClamped, clampedDesiredShoulderPosition != this.desiredShoulderPosition || clampedDesiredWristPosition != this.desiredWristPosition);

                    this.desiredShoulderPosition = clampedDesiredShoulderPosition;
                    this.desiredWristPosition = clampedDesiredWristPosition;
                }
            }
        }

        if (TuningConstants.ARM_USE_SHOULDER_ABSOLUTE_ENCODER_RESET &&
            this.useThroughBoreEncoders &&
            this.shoulderAbsoluteEncoderPosition != null &&
            Helpers.RoughEquals(this.shoulderVelocityAverage, TuningConstants.ZERO, TuningConstants.ARM_SHOULDER_RESET_STOPPED_VELOCITY_THRESHOLD) &&
            Helpers.RoughEquals(this.shoulderPosition, this.desiredShoulderPosition, TuningConstants.ARM_SHOULDER_RESET_AT_POSITION_THRESHOLD) &&
            !Helpers.RoughEquals(this.shoulderPosition, this.shoulderAbsoluteEncoderPosition, TuningConstants.ARM_SHOULDER_RESET_CORRECTION_THRESHOLD) &&
            Helpers.RoughEquals(this.shoulderPosition, this.shoulderAbsoluteEncoderPosition, TuningConstants.ARM_SHOULDER_RESET_DIFFERENCE_MAX) &&
            Helpers.WithinRange(this.shoulderAbsoluteEncoderPosition, TuningConstants.ARM_SHOULDER_MIN_POSITION, TuningConstants.ARM_SHOULDER_MAX_POSITION))
        {
            this.updateCurrShoulderPosition = JumpProtectionReason.Reset;
            this.shoulderPosition = this.shoulderAbsoluteEncoderPosition;
            this.shoulderMotor.setPosition(this.shoulderAbsoluteEncoderPosition);
            this.shoulderMotor.burnFlash();
        }

        if (TuningConstants.ARM_USE_WRIST_ABSOLUTE_ENCODER_RESET &&
            this.useThroughBoreEncoders &&
            this.wristAbsoluteEncoderPosition != null &&
            Helpers.RoughEquals(this.wristVelocityAverage, TuningConstants.ZERO, TuningConstants.ARM_WRIST_RESET_STOPPED_VELOCITY_THRESHOLD) &&
            Helpers.RoughEquals(this.wristPosition, this.desiredWristPosition, TuningConstants.ARM_WRIST_RESET_AT_POSITION_THRESHOLD) &&
            !Helpers.RoughEquals(this.wristPosition, this.wristAbsoluteEncoderPosition, TuningConstants.ARM_WRIST_RESET_CORRECTION_THRESHOLD) &&
            Helpers.RoughEquals(this.wristPosition, this.wristAbsoluteEncoderPosition, TuningConstants.ARM_WRIST_RESET_DIFFERENCE_MAX) &&
            Helpers.WithinRange(this.wristAbsoluteEncoderPosition, TuningConstants.ARM_WRIST_MIN_POSITION, TuningConstants.ARM_WRIST_MAX_POSITION))
        {
            this.updateCurrWristPosition = JumpProtectionReason.Reset;
            this.wristPosition = this.wristAbsoluteEncoderPosition;
            this.wristMotor.setPosition(this.wristAbsoluteEncoderPosition);
            this.wristMotor.burnFlash();
        }
        else if (TuningConstants.ARM_RESET_WRIST_WHEN_LIMIT_SWITCH_HIT &&
            this.wristLimitSwitchHit &&
            Helpers.RoughEquals(this.wristVelocityAverage, TuningConstants.ZERO, TuningConstants.ARM_WRIST_RESET_STOPPED_VELOCITY_THRESHOLD) &&
            Helpers.RoughEquals(this.shoulderVelocityAverage, TuningConstants.ZERO, TuningConstants.ARM_SHOULDER_RESET_STOPPED_VELOCITY_THRESHOLD) &&
            Helpers.RoughEquals(this.desiredWristPosition, TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION) &&
            !Helpers.RoughEquals(this.wristPosition, TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION, 1.5))
        {
            this.updateCurrWristPosition = JumpProtectionReason.Reset;
            this.wristPosition = TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION;
            this.wristMotor.setPosition(TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);
            this.wristMotor.burnFlash();
        }

        // TMP
        this.currentDesiredShoulderPosition = this.desiredShoulderPosition;
        this.currentDesiredWristPosition = this.desiredWristPosition + wristSlopAdjustment;
        if (TuningConstants.ARM_USE_MM)
        {
            // shoulder Trapezoidal Motion Profile follower
            TrapezoidProfile.State shoulderCurr = this.shoulderTMPCurrState;
            TrapezoidProfile.State shoulderGoal = this.shoulderTMPGoalState;

            shoulderGoal.updatePosition(this.currentDesiredShoulderPosition);
            if (this.updateCurrShoulderPosition != JumpProtectionReason.None)
            {
                shoulderCurr.updatePosition(this.shoulderPosition);
                if (this.updateCurrShoulderPosition == JumpProtectionReason.Stall ||
                    this.updateCurrShoulderPosition == JumpProtectionReason.Startup ||
                    this.updateCurrShoulderPosition == JumpProtectionReason.Reset)
                {
                    shoulderCurr.setVelocity(0.0);
                }

                this.updateCurrShoulderPosition = JumpProtectionReason.None;
            }

            if (this.shoulderTrapezoidMotionProfile.update(elapsedTime, shoulderCurr, shoulderGoal))
            {
                this.currentDesiredShoulderPosition = shoulderCurr.getPosition();
            }

            // wrist Trapezoidal Motion Profile follower
            TrapezoidProfile.State wristCurr = this.wristTMPCurrState;
            TrapezoidProfile.State wristGoal = this.wristTMPGoalState;

            wristGoal.updatePosition(this.currentDesiredWristPosition);
            if (this.updateCurrWristPosition != JumpProtectionReason.None)
            {
                wristCurr.updatePosition(this.wristPosition);
                if (this.updateCurrWristPosition == JumpProtectionReason.Stall ||
                    this.updateCurrWristPosition == JumpProtectionReason.Startup ||
                    this.updateCurrWristPosition == JumpProtectionReason.Reset)
                {
                    wristCurr.setVelocity(0.0);
                }

                this.updateCurrWristPosition = JumpProtectionReason.None;
            }

            if (this.wristTrapezoidMotionProfile.update(elapsedTime, wristCurr, wristGoal))
            {
                this.currentDesiredWristPosition = wristCurr.getPosition();
            }
        }

        // IK adjustments
        this.kinematicsLimitedAngles.set(this.lastLegalShoulderPosition, this.lastLegalWristPosition);
        boolean ikChangedPosition =
            this.armKinematicsCalculator.calculateArmLimits(
                this.currentDesiredShoulderPosition,
                this.currentDesiredWristPosition,
                this.kinematicsLimitedAngles);
        if (TuningConstants.ARM_USE_IK_CONSTRAINTS && mode != RobotMode.Test)
        {
            double ikFixedShoulderPosition = this.kinematicsLimitedAngles.getFirst();
            double ikFixedWristPosition = this.kinematicsLimitedAngles.getSecond();

            ExceptionHelpers.Assert(ikChangedPosition || ikFixedShoulderPosition == this.currentDesiredShoulderPosition, "Shoulder %.2f should equal %s2f when we are not changing the position due to IK", ikFixedShoulderPosition, this.currentDesiredShoulderPosition);
            ExceptionHelpers.Assert(ikChangedPosition || ikFixedWristPosition == this.currentDesiredWristPosition, "Wrist %.2f should equal %s2f when we are not changing the position due to IK", ikFixedWristPosition, this.currentDesiredShoulderPosition);

            // if we're not using the updated position using the trapezoidal motion profile, then we will need to adjust
            // the "current position" within the trapezoidal motion profile during the next update loop
            if (ikChangedPosition)
            {
                this.updateCurrShoulderPosition = this.currentDesiredShoulderPosition == ikFixedShoulderPosition ? JumpProtectionReason.None : JumpProtectionReason.IK;
                this.updateCurrWristPosition = this.currentDesiredWristPosition == ikFixedWristPosition ? JumpProtectionReason.None : JumpProtectionReason.IK;
            }

            this.currentDesiredShoulderPosition = ikFixedShoulderPosition;
            this.currentDesiredWristPosition = ikFixedWristPosition;
        }
        else
        {
            this.armKinematicsCalculator.calculate(this.currentDesiredShoulderPosition, this.currentDesiredWristPosition);
        }

        this.lastLegalWristPosition = this.currentDesiredWristPosition;
        this.lastLegalShoulderPosition = this.currentDesiredShoulderPosition;

        // GRAVITY COMPENSATION
        double shoulderFeedForward;
        double shoulderPowerStallingThreshold;
        if (TuningConstants.ARM_USE_GRAVITY_COMPENSATION)
        {
            shoulderFeedForward = this.interpolator.sample(this.currentDesiredShoulderPosition, this.currentDesiredWristPosition);
            ExceptionHelpers.Assert(shoulderFeedForward < TuningConstants.ARM_MAX_GRAVITY_COMPENSATION, "Expect gravity compensation to be less than %.2f, actual %.2f", TuningConstants.ARM_MAX_GRAVITY_COMPENSATION, shoulderFeedForward);
            shoulderFeedForward = Helpers.EnforceRange(shoulderFeedForward, 0.0, TuningConstants.ARM_MAX_GRAVITY_COMPENSATION);
            shoulderPowerStallingThreshold = TuningConstants.BATTERY_AVERAGE_EXPECTED_VOLTAGE * (TuningConstants.ARM_SHOULDER_STALLED_CURRENT_BUFFER + shoulderFeedForward * TuningConstants.PERCENT_OUTPUT_MULTIPLIER);
        }
        else
        {
            shoulderFeedForward = 0.0;
            shoulderPowerStallingThreshold = TuningConstants.ARM_SHOULDER_STALLED_POWER_THRESHOLD;
        }

        if (TuningConstants.ARM_STALL_PROTECTION_ENABLED)
        {
            // if we've past the velocity tracking duration since last desired position change we're using more power than expected and we're not moving that much
            // then either reset position (if past & present values tell us were trying to reset) and say were stalled or just say were stalled

            if (currTime > this.shoulderSetpointChangedTime + TuningConstants.ARM_SHOULDER_VELOCITY_TRACKING_DURATION &&
                this.shoulderPowerAverage >= shoulderPowerStallingThreshold &&
                Math.abs(this.shoulderVelocityAverage) <= TuningConstants.ARM_SHOULDER_STALLED_VELOCITY_THRESHOLD)
            {
                if (Helpers.RoughEquals(this.shoulderPosition, TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD) &&
                    Helpers.RoughEquals(this.desiredShoulderPosition, TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD))
                {
                    this.desiredShoulderPosition = TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION;
                    this.shoulderMotor.setPosition(this.desiredShoulderPosition);
                }

                this.shoulderStalled = true;
                this.updateCurrShoulderPosition = JumpProtectionReason.Stall;
            }
            else if (this.shoulderStalled)
            {
                this.updateCurrShoulderPosition = JumpProtectionReason.Stall;
            }

            if (currTime > this.wristSetpointChangedTime + TuningConstants.ARM_WRIST_VELOCITY_TRACKING_DURATION &&
                this.wristPowerAverage >= TuningConstants.ARM_WRIST_STALLED_POWER_THRESHOLD &&
                Math.abs(this.wristVelocityAverage) <= TuningConstants.ARM_WRIST_STALLED_VELOCITY_THRESHOLD)
            {
                if (Helpers.RoughEquals(this.wristPosition, TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_GOAL_THRESHOLD) &&
                    Helpers.RoughEquals(this.desiredWristPosition, TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_WRIST_GOAL_THRESHOLD))
                {
                    this.desiredWristPosition = TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION;
                    this.wristMotor.setPosition(this.desiredWristPosition);
                }

                this.wristStalled = true;
                this.updateCurrWristPosition = JumpProtectionReason.Stall;
            }
            else if (this.wristStalled)
            {
                this.updateCurrWristPosition = JumpProtectionReason.Stall;
            }
        }

        if (!useShoulderSimpleMode)
        {
            if (this.shoulderStalled ||
                (TuningConstants.ARM_SHOULDER_STOP_WHEN_BOTTOM &&
                    mode != RobotMode.Autonomous &&
                    Helpers.RoughEquals(this.shoulderPosition, this.desiredShoulderPosition, 1.0) &&
                    Helpers.RoughEquals(this.desiredShoulderPosition, TuningConstants.ARM_SHOULDER_MIN_POSITION, 1.0)))
            {
                this.shoulderMotor.stop();
            }
            else
            {
                // Actually, we do...  We don't use a velocity feed-forward, so we can see some large distances here.  ~5deg for shoulder
                // ExceptionHelpers.Assert(
                //     TuningConstants.ARM_USE_MM && Math.abs(this.currentDesiredShoulderPosition - this.shoulderPosition) < 3.0 * TuningConstants.ARM_SHOULDER_MOTOR_TMP_PID_CRUISE_VELOC * elapsedTime + 2.0,
                //     "don't expect shoulder jumps of this much! %.2f -> %.2f (%.2f)",
                //     this.shoulderPosition,
                //     this.currentDesiredShoulderPosition,
                //     3.0 * TuningConstants.ARM_SHOULDER_MOTOR_TMP_PID_CRUISE_VELOC * elapsedTime + 2.0);
                this.shoulderMotor.set(
                    SparkMaxControlMode.Position,
                    this.currentDesiredShoulderPosition,
                    shoulderFeedForward);
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
                // Actually, we do...  We don't use a velocity feed-forward, so we can see some large distances here.  ~40deg for wrist
                // ExceptionHelpers.Assert(
                //     TuningConstants.ARM_USE_MM && Math.abs(this.currentDesiredWristPosition - this.wristPosition) < 3.0 * TuningConstants.ARM_WRIST_MOTOR_TMP_PID_CRUISE_VELOC * elapsedTime + 2.0,
                //     "don't expect wrist jumps of this much! %.2f -> %.2f (%.2f)",
                //     this.wristPosition,
                //     this.currentDesiredWristPosition,
                //     3.0 * TuningConstants.ARM_WRIST_MOTOR_TMP_PID_CRUISE_VELOC * elapsedTime + 2.0);
                this.wristMotor.set(
                    SparkMaxControlMode.Position,
                    this.currentDesiredWristPosition,
                    0.0);
            }
        }
        else
        {
            this.wristMotor.set(SparkMaxControlMode.PercentOutput, wristPower);
        }

        this.logger.logNumber(LoggingKey.ArmShoulderOutput, this.shoulderMotor.getOutput());
        this.logger.logNumber(LoggingKey.ArmWristOutput, this.wristMotor.getOutput());
        this.logger.logBoolean(LoggingKey.ArmShoulderStalled, this.shoulderStalled);
        this.logger.logBoolean(LoggingKey.ArmWristStalled, this.wristStalled);

        this.logger.logNumber(LoggingKey.ArmShoulderSetpointTMP, this.currentDesiredShoulderPosition);
        this.logger.logNumber(LoggingKey.ArmWristSetpointTMP, this.currentDesiredWristPosition);
        this.logger.logNumber(LoggingKey.ArmShoulderSetpoint, this.desiredShoulderPosition);
        this.logger.logNumber(LoggingKey.ArmWristSetpoint, this.desiredWristPosition);
        this.armKinematicsCalculator.logValues(this.logger);

        this.prevTime = currTime;
    }

    @Override
    public void stop()
    {
        this.shoulderMotor.stop();
        this.wristMotor.stop();
        this.prevTime = 0.0;

        if (TuningConstants.ARM_USE_COAST_ON_DISABLE)
        {
            this.wristMotor.setNeutralMode(MotorNeutralMode.Coast);
            this.wristMotor.burnFlash();
        }

        this.wasEnabled = false;

        this.updateCurrShoulderPosition = JumpProtectionReason.Startup;
        this.updateCurrWristPosition = JumpProtectionReason.Startup;
    }

    public double[] getWristJointAbsPosition()
    {
        return this.armKinematicsCalculator.getWristJointAbsPosition();
    }

    public boolean getStuckInPosition()
    {
        return this.armKinematicsCalculator.getStuckInPosition();
    }

    public double getAbsoluteAngleOfShot()
    {
        return this.armKinematicsCalculator.getAbsoluteAngleOfShot();
    }

    public boolean getInSimpleMode()
    {
        return this.inSimpleMode;
    }

    public boolean getShoulderStalled()
    {
        return this.shoulderStalled;
    }

    public boolean getWristStalled()
    {
        return this.wristStalled;
    }

    public double getShoulderPosition()
    {
        return this.shoulderPosition;
    }

    public double getWristPosition()
    {
        return this.wristPosition;
    }

    public double getShoulderPowerAverage()
    {
        return this.shoulderPowerAverage;
    }

    public double getWristPowerAverage()
    {
        return this.wristPowerAverage;
    }

    public double getWristVelocityAverage()
    {
        return this.wristVelocityAverage;
    }

    public double getShoulderVelocityAverage()
    {
        return this.shoulderVelocityAverage;
    }

    public boolean getWristLimitSwitchStatus()
    {
        return this.wristLimitSwitchHit;
    }
}
