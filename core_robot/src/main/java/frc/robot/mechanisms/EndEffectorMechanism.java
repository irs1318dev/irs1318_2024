package frc.robot.mechanisms;

import frc.robot.*;
import frc.lib.driver.*;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.*;
import frc.robot.driver.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class EndEffectorMechanism implements IMechanism
{
    private static final int DefaultPidSlotId = 0;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final ITalonSRX intakeMotor;

    private final ISparkMax nearFlywheelMotor;
    private final ISparkMax farFlywheelMotor;

    private final IAnalogInput throughBeamSensor;

    private double intakeMotorVelocity;

    private double nearFlywheelPosition;
    private double nearFlywheelVelocity;
    private double nearFlywheelError;

    private double farFlywheelPosition;
    private double farFlywheelVelocity;
    private double farFlywheelError;

    private double throughBeamSensorValue;
    private boolean throughBeamBroken;

    private double nearFlywheelSetpoint;
    private double farFlywheelSetpoint;

    public enum EffectorState
    {
        Off,
        Intaking,
        Outtaking,
        Shooting
    };

    private EffectorState currentEffectorState;

    private boolean useShootAnywayMode;
    private boolean useIntakeForceSpin;
    private double shootingStartTime;

    @Inject
    public EndEffectorMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider,
        ITimer timer)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;

        // INTAKE MOTOR
        this.intakeMotor = provider.getTalonSRX(ElectronicsConstants.INTAKE_MOTOR_CAN_ID);
        this.intakeMotor.setMotorOutputSettings(TuningConstants.INTAKE_MOTOR_INVERT_OUTPUT, MotorNeutralMode.Brake);
        this.intakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);

        // NEAR FLYWHEEL MOTOR
        this.nearFlywheelMotor = provider.getSparkMax(ElectronicsConstants.SHOOTER_NEAR_FLYWHEEL_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.nearFlywheelMotor.setRelativeEncoder();
        this.nearFlywheelMotor.setInvertOutput(TuningConstants.NEAR_SHOOTER_MOTOR_INVERT_OUTPUT);
        this.nearFlywheelMotor.setNeutralMode(MotorNeutralMode.Coast);
        this.nearFlywheelMotor.setControlMode(SparkMaxControlMode.PercentOutput);

        this.nearFlywheelMotor.setPIDF(
            TuningConstants.SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KP,
            TuningConstants.SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KI,
            TuningConstants.SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KD,
            TuningConstants.SHOOTER_NEAR_FLYWHEEL_MOTOR_PID_KF,
            DefaultPidSlotId);

        this.nearFlywheelMotor.setVelocityConversionFactor(HardwareConstants.SHOOTER_NEAR_FLYWHEEL_TICK_DISTANCE);
        this.nearFlywheelMotor.setCurrentLimit(TuningConstants.FLYWHEEL_STALL_LIMIT, TuningConstants.FLYWHEEL_FREE_LIMIT, TuningConstants.FLYWHEEL_RPM_LIMIT);
        this.nearFlywheelMotor.setFeedbackFramePeriod(SparkMaxPeriodicFrameType.Status0, TuningConstants.FLYWHEEL_SENSOR_FRAME_PERIOD_MS);
        this.nearFlywheelMotor.setSelectedSlot(DefaultPidSlotId);

        this.nearFlywheelMotor.burnFlash();

        // FAR FLYWHEEL MOTOR
        this.farFlywheelMotor = provider.getSparkMax(ElectronicsConstants.SHOOTER_FAR_FLYWHEEL_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.farFlywheelMotor.setRelativeEncoder();
        this.farFlywheelMotor.setInvertOutput(TuningConstants.FAR_SHOOTER_MOTOR_INVERT_OUTPUT);
        this.farFlywheelMotor.setNeutralMode(MotorNeutralMode.Coast);
        this.farFlywheelMotor.setControlMode(SparkMaxControlMode.PercentOutput);

        this.farFlywheelMotor.setPIDF(
            TuningConstants.SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KP,
            TuningConstants.SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KI,
            TuningConstants.SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KD,
            TuningConstants.SHOOTER_FAR_FLYWHEEL_MOTOR_PID_KF,
            DefaultPidSlotId);

        this.farFlywheelMotor.setVelocityConversionFactor(HardwareConstants.SHOOTER_FAR_FLYWHEEL_TICK_DISTANCE);
        this.farFlywheelMotor.setCurrentLimit(TuningConstants.FLYWHEEL_STALL_LIMIT, TuningConstants.FLYWHEEL_FREE_LIMIT, TuningConstants.FLYWHEEL_RPM_LIMIT);
        this.farFlywheelMotor.setFeedbackFramePeriod(SparkMaxPeriodicFrameType.Status0, TuningConstants.FLYWHEEL_SENSOR_FRAME_PERIOD_MS);
        this.farFlywheelMotor.setSelectedSlot(DefaultPidSlotId);

        this.farFlywheelMotor.burnFlash();

        // THROUGH BEAM
        this.throughBeamSensor = provider.getAnalogInput(ElectronicsConstants.INTAKE_THROUGHBEAM_ANALOG_INPUT);

        this.useShootAnywayMode = false;
        this.useIntakeForceSpin = false;
        this.shootingStartTime = 0.0;

        this.currentEffectorState = EffectorState.Off;
    }

    @Override
    public void readSensors()
    {
        this.intakeMotorVelocity = this.intakeMotor.getVelocity();
        this.logger.logNumber(LoggingKey.IntakeMotorVelocity, this.intakeMotorVelocity);

        this.nearFlywheelPosition = this.nearFlywheelMotor.getPosition();
        this.nearFlywheelVelocity = this.nearFlywheelMotor.getVelocity();
        this.nearFlywheelError = this.nearFlywheelMotor.getVelocity() - this.nearFlywheelSetpoint;

        this.logger.logNumber(LoggingKey.ShooterNearFlywheelPosition, this.nearFlywheelPosition);
        this.logger.logNumber(LoggingKey.ShooterNearFlywheelVelocity, this.nearFlywheelVelocity);
        this.logger.logNumber(LoggingKey.ShooterNearFlywheelError, this.nearFlywheelError);

        this.farFlywheelPosition = this.farFlywheelMotor.getPosition();
        this.farFlywheelVelocity = this.farFlywheelMotor.getVelocity();
        this.farFlywheelError = this.farFlywheelMotor.getVelocity() - this.farFlywheelSetpoint;

        this.logger.logNumber(LoggingKey.ShooterFarFlywheelPosition, this.farFlywheelPosition);
        this.logger.logNumber(LoggingKey.ShooterFarFlywheelVelocity, this.farFlywheelVelocity);
        this.logger.logNumber(LoggingKey.ShooterFarFlywheelError, this.farFlywheelError);

        this.logger.logBoolean(LoggingKey.ShooterSpunUp, this.isFlywheelSpunUp());
        this.throughBeamSensorValue = this.throughBeamSensor.getVoltage();
        this.throughBeamBroken = this.throughBeamSensorValue < TuningConstants.INTAKE_THROUGHBEAM_CUTOFF;

        this.logger.logNumber(LoggingKey.IntakeThroughBeamSensorValue, this.throughBeamSensorValue);
        this.logger.logBoolean(LoggingKey.IntakeThroughBeamBroken, this.throughBeamBroken);
    }

    @Override
    public void update(RobotMode mode)
    {
        double currTime = this.timer.get();

        // FLYWHEEL LOGIC
        double flywheelMotorPower = this.driver.getAnalog(AnalogOperation.EndEffectorFlywheelMotorPower);
        double nearFlywheelVelocityGoal = this.driver.getAnalog(AnalogOperation.EndEffectorNearFlywheelVelocityGoal); // This value should be calculated and in RPM
        double farFlywheelVelocityGoal = this.driver.getAnalog(AnalogOperation.EndEffectorFarFlywheelVelocityGoal); // This value should be calculated and in RPM
        double noteOut = this.driver.getAnalog(AnalogOperation.EndEffectorGetNoteOut);

        if (flywheelMotorPower != TuningConstants.ZERO)
        {
            this.nearFlywheelSetpoint = this.nearFlywheelVelocity;
            this.farFlywheelSetpoint = this.farFlywheelVelocity;

            this.nearFlywheelMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            this.farFlywheelMotor.setControlMode(SparkMaxControlMode.PercentOutput);

            this.nearFlywheelMotor.set(flywheelMotorPower);
            this.farFlywheelMotor.set(flywheelMotorPower);

            this.logger.logNumber(LoggingKey.ShooterFlywheelPower, flywheelMotorPower);
        }
        else if (noteOut != TuningConstants.ZERO)
        {
            this.nearFlywheelSetpoint = this.nearFlywheelVelocity;
            this.farFlywheelSetpoint = this.farFlywheelVelocity;

            this.nearFlywheelMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            this.farFlywheelMotor.setControlMode(SparkMaxControlMode.PercentOutput);

            this.nearFlywheelMotor.set(noteOut);
            this.farFlywheelMotor.set(-noteOut);
        }
        else if (nearFlywheelVelocityGoal != TuningConstants.MAGIC_NULL_VALUE && farFlywheelVelocityGoal != TuningConstants.MAGIC_NULL_VALUE)
        {
            this.nearFlywheelSetpoint = nearFlywheelVelocityGoal;
            this.farFlywheelSetpoint = farFlywheelVelocityGoal;

            this.nearFlywheelMotor.setControlMode(SparkMaxControlMode.Velocity);
            this.farFlywheelMotor.setControlMode(SparkMaxControlMode.Velocity);

            this.nearFlywheelMotor.set(this.nearFlywheelSetpoint);
            this.farFlywheelMotor.set(this.farFlywheelSetpoint);

            this.logger.logNumber(LoggingKey.ShooterFlywheelPower, TuningConstants.MAGIC_NULL_VALUE);
        }
        else
        {
            this.nearFlywheelSetpoint = TuningConstants.ZERO;
            this.farFlywheelSetpoint = TuningConstants.ZERO;

            this.nearFlywheelMotor.stop();
            this.farFlywheelMotor.stop();

            this.logger.logNumber(LoggingKey.ShooterFlywheelPower, TuningConstants.MAGIC_NULL_VALUE);
        }

        this.logger.logNumber(LoggingKey.ShooterNearFlywheelDesiredVelocity, this.nearFlywheelSetpoint);
        this.logger.logNumber(LoggingKey.ShooterFarFlywheelDesiredVelocity, this.farFlywheelSetpoint);

        this.useShootAnywayMode = this.driver.getDigital(DigitalOperation.ShooterShootAnywayMode);
        // if (this.driver.getDigital(DigitalOperation.ShooterShootAnywayMode))
        // {
        //     this.useShootAnywayMode = true;
        // }
        // else if (this.driver.getDigital(DigitalOperation.ShooterDisableShootAnywayMode))
        // {
        //     this.useShootAnywayMode = false;
        // }

        if (this.driver.getDigital(DigitalOperation.IntakeForceSpinOn))
        {
            this.useIntakeForceSpin = true;
        }
        else if (this.driver.getDigital(DigitalOperation.IntakeForceSpinOff))
        {
            this.useIntakeForceSpin = false;
        }

        if (this.driver.getDigital(DigitalOperation.IntakeForceOnAndIntakeIn))
        {
            this.useIntakeForceSpin = true;
            this.currentEffectorState = EffectorState.Intaking;
        }

        // STATE SWITCHING
        boolean intakeOutSlow = this.driver.getDigital(DigitalOperation.IntakeOutSlow);
        switch (this.currentEffectorState)
        {
            case Off:
                // Stay off if told to
                if (this.driver.getDigital(DigitalOperation.IntakeForceStop))
                {
                    this.currentEffectorState = EffectorState.Off;
                }
                // Start intaking when told to and the through-beam isn't broken, or we are ignoring through-beam
                else if (this.driver.getDigital(DigitalOperation.IntakeIn) &&
                    (!this.throughBeamBroken || this.useIntakeForceSpin))
                {
                    this.currentEffectorState = EffectorState.Intaking;
                }
                // Start shooting if told to, and flywheel is spun up or we don't care about spun up
                else if (this.driver.getDigital(DigitalOperation.ShooterFeedRing) &&
                    (this.isFlywheelSpunUp() || this.useShootAnywayMode))
                {
                    this.currentEffectorState = EffectorState.Shooting;
                    this.shootingStartTime = currTime;
                }
                // Start outtaking when told to
                else if (this.driver.getDigital(DigitalOperation.IntakeOut) ||
                    intakeOutSlow)
                {
                    this.currentEffectorState = EffectorState.Outtaking;
                }
                // otherwise, remain in off state
                else
                {
                    this.currentEffectorState = EffectorState.Off;
                }

                break;

            case Intaking:
                // Stop if forced to
                if (this.driver.getDigital(DigitalOperation.IntakeForceStop))
                {
                    this.currentEffectorState = EffectorState.Off;
                }
                // continue intaking when told to and the through-beam isn't broken, or we are ignoring through-beam
                else if (this.driver.getDigital(DigitalOperation.IntakeIn) &&
                    (!this.throughBeamBroken || this.useIntakeForceSpin))
                {
                    this.currentEffectorState = EffectorState.Intaking;
                }
                // shoot if told to, and were not intaking
                else if (this.driver.getDigital(DigitalOperation.ShooterFeedRing) &&
                    (this.isFlywheelSpunUp() || this.useShootAnywayMode))
                {
                    this.currentEffectorState = EffectorState.Shooting;
                    this.shootingStartTime = currTime;
                }
                // outtake if told to
                else if (this.driver.getDigital(DigitalOperation.IntakeOut) ||
                    intakeOutSlow)
                {
                    this.currentEffectorState = EffectorState.Outtaking;
                }
                // if no button is pressed, through-beam is broken (and not forcing), switch to off
                else // if (this.throughBeamBroken && !this.driver.getDigital(DigitalOperation.IntakeForceIn))
                {
                    this.currentEffectorState = EffectorState.Off;
                }

                break;

            case Shooting:
                // stop if forced to
                if (this.driver.getDigital(DigitalOperation.IntakeForceStop))
                {
                    this.currentEffectorState = EffectorState.Off;
                }
                // start intaking when told to and the through-beam isn't broken, or we are ignoring through-beam
                else if (this.driver.getDigital(DigitalOperation.IntakeIn) &&
                    (!this.throughBeamBroken || this.useIntakeForceSpin))
                {
                    this.currentEffectorState = EffectorState.Intaking;
                }
                // continue shooting if told to (and were not intaking)
                else if (this.driver.getDigital(DigitalOperation.ShooterFeedRing) &&
                    (currTime -  this.shootingStartTime <= TuningConstants.EFFECTOR_SHOOTING_DURATION))
                {
                    this.currentEffectorState = EffectorState.Shooting;
                }
                // outtake if told to
                else if (this.driver.getDigital(DigitalOperation.IntakeOut) ||
                    intakeOutSlow)
                {
                    this.currentEffectorState = EffectorState.Outtaking;
                }
                // if no button is pressed, or our shooting duration has expired, switch to off
                else
                {
                    this.currentEffectorState = EffectorState.Off;
                }

                break;

            case Outtaking:
                // stop if forced to
                if (this.driver.getDigital(DigitalOperation.IntakeForceStop))
                {
                    this.currentEffectorState = EffectorState.Off;
                }
                // start intaking when told to and the through-beam isn't broken, or we are ignoring through-beam
                else if (this.driver.getDigital(DigitalOperation.IntakeIn) &&
                    (!this.throughBeamBroken || this.useIntakeForceSpin))
                {
                    this.currentEffectorState = EffectorState.Intaking;
                }
                // feed ring if told to
                else if (this.driver.getDigital(DigitalOperation.ShooterFeedRing) &&
                    (this.isFlywheelSpunUp() || this.useShootAnywayMode))
                {
                    this.currentEffectorState = EffectorState.Shooting;
                    this.shootingStartTime = currTime;
                }
                // outtake if told to
                else if (this.driver.getDigital(DigitalOperation.IntakeOut) ||
                    intakeOutSlow)
                {
                    this.currentEffectorState = EffectorState.Outtaking;
                }
                // if no button is pressed, switch to off
                else
                {
                    this.currentEffectorState = EffectorState.Off;
                }

                break;
        }
        
        // STATE CONTROL
        double intakePower;
        switch (this.currentEffectorState)
        {
            case Intaking:
                intakePower = mode == RobotMode.Autonomous ? TuningConstants.EFFECTOR_INTAKE_IN_AUTO_POWER : TuningConstants.EFFECTOR_INTAKE_IN_POWER;
                break;

            case Outtaking:
                intakePower = intakeOutSlow ? TuningConstants.EFFECTOR_INTAKE_OUT_SLOW_POWER : TuningConstants.EFFECTOR_INTAKE_OUT_POWER;
                break;

            case Shooting:
                intakePower = TuningConstants.EFFECTOR_INTAKE_FEED_SHOOTER_POWER;
                break;

            default:
            case Off:
                intakePower = TuningConstants.ZERO;
                break;
        }

        this.intakeMotor.set(intakePower);
        this.logger.logNumber(LoggingKey.IntakeMotorPercentOutput, intakePower);
    }

    @Override
    public void stop()
    {
        this.intakeMotor.stop();
        this.nearFlywheelMotor.stop();
        this.farFlywheelMotor.stop();

        this.shootingStartTime = 0.0;
    }

    public double getNearFlywheelSetpoint()
    {
        return this.nearFlywheelSetpoint;
    }

    public double getFarFlywheelSetpoint()
    {
        return this.farFlywheelSetpoint;
    }

    public boolean isFlywheelSpunUp()
    {
        return this.farFlywheelSetpoint > 0.0 &&
               this.nearFlywheelSetpoint > 0.0 &&
               Math.abs(this.farFlywheelError) <= TuningConstants.FLYWHEEL_ALLOWABLE_ERROR_RANGE &&
               Math.abs(this.nearFlywheelError) <= TuningConstants.FLYWHEEL_ALLOWABLE_ERROR_RANGE;
    }

    public boolean hasGamePiece()
    {
        return this.throughBeamBroken;
    }

    public EffectorState getEndEffectorState() 
    {
        return this.currentEffectorState;
    }

    public double getFarFlywheelVelocity() {
        return farFlywheelVelocity;
    }

    public double getNearFlywheelVelocity() {
        return nearFlywheelVelocity;
    }
}