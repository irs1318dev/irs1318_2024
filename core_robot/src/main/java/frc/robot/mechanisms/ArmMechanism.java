package frc.robot.mechanisms;

import frc.lib.controllers.TrapezoidProfile;
import frc.lib.driver.IDriver;
import frc.lib.filters.FloatingAverageCalculator;
import frc.lib.helpers.BilinearInterpolator;
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
    private final ISparkMax wristMotor;

    private double desiredShoulderPosition;
    private double desiredWristPosition;

    private double shoulderPosition;
    private double shoulderVelocity;

    private double wristPosition;
    private double wristVelocity;

    private double shoulderError;
    private double wristError;

    private double shoulderSetpointChangedTime;
    private double wristSetpointChangedTime;
    private boolean shoulderStalled;
    private boolean wristStalled;
    
    private boolean wristLimitSwitchHit;

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
    private boolean updateCurrWristPosition;
    private boolean updateCurrShoulderPosition;

    @Inject
    public ArmMechanism(IRobotProvider provider, IDriver driver, LoggingManager logger, ITimer timer, PowerManager powerManager)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;
        this.powerManager = powerManager;

        this.inSimpleMode = TuningConstants.ARM_USE_SIMPLE_MODE;
        this.updateCurrShoulderPosition = true;
        this.updateCurrWristPosition = true;

        this.shoulderMotor = provider.getSparkMax(ElectronicsConstants.ARM_SHOULDER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.wristMotor = provider.getSparkMax(ElectronicsConstants.ARM_WRIST_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);

        this.shoulderMotor.setRelativeEncoder();
        this.shoulderMotor.setPositionConversionFactor(HardwareConstants.ARM_SHOULDER_TICK_DISTANCE);
        this.shoulderMotor.setVelocityConversionFactor(HardwareConstants.ARM_SHOULDER_TICK_DISTANCE);
        this.shoulderMotor.setInvertOutput(TuningConstants.ARM_SHOULDER_MOTOR_INVERT_OUTPUT);
        this.shoulderMotor.setPosition(TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION);
        this.shoulderMotor.setNeutralMode(MotorNeutralMode.Brake);

        this.wristMotor.setRelativeEncoder();
        this.wristMotor.setForwardLimitSwitch(ElectronicsConstants.ARM_WRIST_LIMIT_SWITCH_ENABLED, ElectronicsConstants.ARM_WRIST_LIMIT_SWITCH_NORMALLY_OPEN);
        this.wristMotor.setPositionConversionFactor(HardwareConstants.ARM_WRIST_TICK_DISTANCE);
        this.wristMotor.setVelocityConversionFactor(HardwareConstants.ARM_WRIST_TICK_DISTANCE);
        this.wristMotor.setInvertOutput(TuningConstants.ARM_WRIST_MOTOR_INVERT_OUTPUT);
        this.wristMotor.setPosition(TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION);
        this.wristMotor.setNeutralMode(MotorNeutralMode.Coast);

        if (TuningConstants.ARM_USE_MM)
        {
            this.shoulderMotor.setPIDF(
                TuningConstants.ARM_SHOULDER_POSITIONAL_TMP_PID_KP,
                TuningConstants.ARM_SHOULDER_POSITIONAL_TMP_PID_KI,
                TuningConstants.ARM_SHOULDER_POSITIONAL_TMP_PID_KD,
                TuningConstants.ARM_SHOULDER_POSITIONAL_TMP_PID_KF,
                ArmMechanism.AltPidSlotId);

            this.wristMotor.setPIDF(
                TuningConstants.ARM_WRIST_POSITIONAL_TMP_PID_KP,
                TuningConstants.ARM_WRIST_POSITIONAL_TMP_PID_KI,
                TuningConstants.ARM_WRIST_POSITIONAL_TMP_PID_KD,
                TuningConstants.ARM_WRIST_POSITIONAL_TMP_PID_KF,
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
                TuningConstants.ARM_SHOULDER_TMP_PID_CRUISE_VELOC,
                TuningConstants.ARM_SHOULDER_TMP_PID_ACCEL);
            this.shoulderTMPCurrState = new TrapezoidProfile.State(this.shoulderPosition, 0.0);
            this.shoulderTMPGoalState = new TrapezoidProfile.State(this.shoulderPosition, 0.0);

            this.shoulderMotor.setSelectedSlot(ArmMechanism.AltPidSlotId);

            this.wristTrapezoidMotionProfile = new TrapezoidProfile(
                TuningConstants.ARM_WRIST_TMP_PID_CRUISE_VELOC,
                TuningConstants.ARM_WRIST_TMP_PID_ACCEL);
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
        this.armKinematicsCalculator = new ArmKinematicsCalculator(this.shoulderPosition, this.wristPosition);
        this.kinematicsLimitedAngles = new Pair<Double, Double>(this.shoulderPosition, this.wristPosition);

        this.lastLegalShoulderPosition = TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION;
        this.lastLegalWristPosition = TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION;

        this.wasEnabled = false;
    }

    @Override
    public void readSensors()
    {
        this.shoulderPosition = this.shoulderMotor.getPosition(); // in degrees (conversion to degrees included in setPositionConversionFactor)
        this.shoulderVelocity = this.shoulderMotor.getVelocity(); // in degrees/sec (conversion to degrees included in setVelocityConversionFactor)
        this.shoulderError = this.shoulderPosition - this.desiredShoulderPosition;
        this.wristPosition = this.wristMotor.getPosition(); // convert rotations to degrees
        this.wristVelocity = this.wristMotor.getVelocity(); // convert ticks/100ms to degrees/sec
        this.wristError = this.wristPosition - this.desiredWristPosition;

        this.wristLimitSwitchHit = this.wristMotor.getForwardLimitSwitchStatus();

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
        double elapsedTime = currTime - this.prevTime;

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

            this.updateCurrShoulderPosition = true;
            this.updateCurrWristPosition = true;
        }
        else if (!this.wasEnabled)
        {
            // note - we want to avoid double-burnFlash, so anything we do here we should do within the arm force reset above
            if (TuningConstants.ARM_USE_COAST_ON_DISABLE)
            {
                this.wristMotor.setNeutralMode(MotorNeutralMode.Brake);
                this.wristMotor.burnFlash();
            }

            this.wasEnabled = true;
        }

        boolean armStop = this.driver.getDigital(DigitalOperation.ArmStop);

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
        else if (armStop)
        {
            useWristSimpleMode = true;
            useShoulderSimpleMode = true;

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
                // preset values and resetting
                double newDesiredShoulderPosition = this.driver.getAnalog(AnalogOperation.ArmShoulderPositionSetpoint);
                double newDesiredWristPosition = this.driver.getAnalog(AnalogOperation.ArmWristPositionSetpoint);

                // shooter calculates for abs shot angle not wrist angle
                if (newDesiredWristPosition == TuningConstants.MAGIC_NULL_VALUE)
                {
                    double newDesiredAbsoluteWristAngle = this.driver.getAnalog(AnalogOperation.ArmAbsWristAngle);
                    if (newDesiredAbsoluteWristAngle != TuningConstants.MAGIC_NULL_VALUE)
                    {
                        newDesiredWristPosition = this.armKinematicsCalculator.switchToTheta2(newDesiredAbsoluteWristAngle);
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
                        this.updateCurrShoulderPosition = this.updateCurrShoulderPosition || !Helpers.RoughEquals(this.desiredShoulderPosition, newDesiredShoulderPosition, 0.1);

                        this.shoulderSetpointChangedTime = currTime;
                        this.shoulderStalled = false;

                        this.desiredShoulderPosition = newDesiredShoulderPosition;
                    }

                    if (newDesiredWristPosition != TuningConstants.MAGIC_NULL_VALUE &&
                        (!Helpers.RoughEquals(this.desiredWristPosition, newDesiredWristPosition, 0.1) ||
                         (!Helpers.RoughEquals(this.wristPosition, newDesiredWristPosition, 1.0) && this.wristStalled)))
                    {
                        this.updateCurrWristPosition = this.updateCurrWristPosition || !Helpers.RoughEquals(this.desiredWristPosition, newDesiredWristPosition, 0.1);

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

                // clamp the values to the allowed ranges
                double clampedDesiredShoulderPosition = Helpers.EnforceRange(this.desiredShoulderPosition, TuningConstants.ARM_SHOULDER_MIN_POSITION, TuningConstants.ARM_SHOULDER_MAX_POSITION);
                double clampedDesiredWristPosition = Helpers.EnforceRange(this.desiredWristPosition, TuningConstants.ARM_WRIST_MIN_POSITION, TuningConstants.ARM_WRIST_MAX_POSITION);
                this.logger.logBoolean(LoggingKey.ArmClamped, clampedDesiredShoulderPosition != this.desiredShoulderPosition || clampedDesiredWristPosition != this.desiredWristPosition);

                this.desiredShoulderPosition = clampedDesiredShoulderPosition;
                this.desiredWristPosition = clampedDesiredWristPosition;
            }
        }

        // TMP
        double currentDesiredShoulderPosition = this.desiredShoulderPosition;
        double currentDesiredWristPosition = this.desiredWristPosition;
        if (TuningConstants.ARM_USE_MM)
        {
            // shoulder Trapezoidal Motion Profile follower
            TrapezoidProfile.State shoulderCurr = this.shoulderTMPCurrState;
            TrapezoidProfile.State shoulderGoal = this.shoulderTMPGoalState;

            shoulderGoal.updatePosition(currentDesiredShoulderPosition);
            if (this.updateCurrShoulderPosition)
            {
                shoulderCurr.updatePosition(this.shoulderPosition);
                this.updateCurrShoulderPosition = false;
            }

            if (this.shoulderTrapezoidMotionProfile.update(elapsedTime, shoulderCurr, shoulderGoal))
            {
                currentDesiredShoulderPosition = shoulderCurr.getPosition();
            }

            // wrist Trapezoidal Motion Profile follower
            TrapezoidProfile.State wristCurr = this.wristTMPCurrState;
            TrapezoidProfile.State wristGoal = this.wristTMPGoalState;

            wristGoal.updatePosition(currentDesiredWristPosition);
            if (this.updateCurrWristPosition)
            {
                wristCurr.updatePosition(this.wristPosition);
                this.updateCurrWristPosition = false;
            }

            if (this.wristTrapezoidMotionProfile.update(elapsedTime, wristCurr, wristGoal))
            {
                currentDesiredWristPosition = wristCurr.getPosition();
            }
        }

        // IK adjustments
        this.kinematicsLimitedAngles.set(this.lastLegalShoulderPosition, this.lastLegalWristPosition);
        boolean useDesired =
            this.armKinematicsCalculator.calculateArmLimits(
                currentDesiredShoulderPosition,
                currentDesiredWristPosition,
                this.kinematicsLimitedAngles);
        if (TuningConstants.USE_IK_CONSTRAINTS)
        {
            currentDesiredShoulderPosition = this.kinematicsLimitedAngles.getFirst();
            currentDesiredWristPosition = this.kinematicsLimitedAngles.getSecond();

            // if we're not using the updated position using the trapezoidal motion profile, then we will need to adjust
            // the "current position" within the trapezoidal motion profile during the next update loop
            if (!useDesired)
            {
                // this.updateCurrShoulderPosition = true;
                this.updateCurrWristPosition = true;
            }
        }
        else
        {
            this.armKinematicsCalculator.calculate(currentDesiredShoulderPosition, currentDesiredWristPosition);
        }

        this.lastLegalWristPosition = currentDesiredWristPosition;
        this.lastLegalShoulderPosition = currentDesiredShoulderPosition;

        double powerThreshold = TuningConstants.BATTERY_AVERAGE_EXPECTED_VOLTAGE * TuningConstants.ARM_SHOULDER_STALLED_CURRENT_BUFFER;

        // GRAVITY COMPENSATION
        double feedForward = 0.0;
        if (TuningConstants.ARM_USE_GRAVITY_COMPENSATION)
        {
            feedForward = this.interpolator.sample(currentDesiredShoulderPosition, currentDesiredWristPosition);
            powerThreshold += feedForward * TuningConstants.PERCENT_OUTPUT_MULTIPLIER;
        }

        if (TuningConstants.ARM_STALL_PROTECTION_ENABLED)
        {
            // if we've past the velocity tracking duration since last desired position change we're using more power than expected and we're not moving that much
            // then either reset position (if past & present values tell us were trying to reset) and say were stalled or just say were stalled
            
            if (currTime > this.shoulderSetpointChangedTime + TuningConstants.ARM_SHOULDER_VELOCITY_TRACKING_DURATION &&
                this.shoulderPowerAverage >= powerThreshold &&
                Math.abs(this.shoulderVelocityAverage) <= TuningConstants.ARM_SHOULDER_STALLED_VELOCITY_THRESHOLD)
            {
                if (Helpers.RoughEquals(this.shoulderPosition, TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD) &&
                    Helpers.RoughEquals(this.desiredShoulderPosition, TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION, TuningConstants.ARM_SHOULDER_GOAL_THRESHOLD))
                {
                    this.desiredShoulderPosition = TuningConstants.ARM_SHOULDER_POSITION_STARTING_CONFIGURATION;
                    this.shoulderMotor.setPosition(this.desiredShoulderPosition);
                }

                this.shoulderStalled = true;
                this.updateCurrShoulderPosition = true;
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
                this.updateCurrWristPosition = true;
            }
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
                    SparkMaxControlMode.Position,
                    currentDesiredWristPosition,
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

        this.updateCurrShoulderPosition = true;
        this.updateCurrWristPosition = true;
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
