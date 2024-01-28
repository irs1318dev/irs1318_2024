package frc.robot.mechanisms;

 import frc.robot.*;
 import frc.lib.driver.*;
 import frc.lib.helpers.Helpers;
 import frc.lib.mechanisms.*;
 import frc.lib.robotprovider.*;
 import frc.robot.driver.*;
 import frc.lib.filters.FloatingAverageCalculator;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class EndEffectorMechanism implements IMechanism 
{
    private static final int DefaultPidSlotId = 0;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;
    private final PowerManager powerManager;

    private double prevTime;

    private final ITalonSRX intakeMotor;
    private final ISparkMax flywheelMotor;

    private final IAnalogInput throughBeamSensor;

    private double intakeMotorVelocity;

    private double flywheelPosition;
    private double flywheelVelocity;
    private double flywheelError;

    private double throughBeamSensorValue;
    private boolean throughBeamBroken;

    private double flywheelSetpoint;

    private enum EffectorState
    {
        Off,
        Intaking,
        Outtaking,
        Shooting
    };

    private EffectorState currentEffectorState;

    private boolean useShootAnywayMode;
    private double outTakeStartTime;
    private double shootingStartTime;

    @Inject
    public EndEffectorMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider,
        PowerManager powerManager,
        ITimer timer)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;
        this.powerManager = powerManager;

        // INTAKE MOTOR
        this.intakeMotor = provider.getTalonSRX(ElectronicsConstants.INTAKE_MOTOR_CAN_ID);
        this.intakeMotor.setMotorOutputSettings(TuningConstants.INTAKE_MOTOR_INVERT_OUTPUT, MotorNeutralMode.Brake);
        this.intakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);

        // FLYWHEEL MOTOR
        this.flywheelMotor = provider.getSparkMax(ElectronicsConstants.FLYWHEEL_MOTOR_CAN_ID, SparkMaxMotorType.Brushed);
        this.flywheelMotor.setRelativeEncoder();
        this.flywheelMotor.setInvertSensor(TuningConstants.SHOOTER_MOTOR_INVERT_SENSOR);
        this.flywheelMotor.setInvertOutput(TuningConstants.SHOOTER_MOTOR_INVERT_OUTPUT);
        this.flywheelMotor.setNeutralMode(MotorNeutralMode.Coast);

        this.flywheelMotor.setPIDF(
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KP, 
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KI, 
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KD, 
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KF, 
            DefaultPidSlotId);

        this.flywheelMotor.setVelocityConversionFactor(TuningConstants.SHOOTER_FLYWHEEL_CONVERSION_FACTOR);
        this.flywheelMotor.setCurrentLimit(TuningConstants.FLYWHEEL_STALL_LIMIT, TuningConstants.FLYWHEEL_FREE_LIMIT, TuningConstants.FLYWHEEL_RPM_LIMIT);
        this.flywheelMotor.setFeedbackFramePeriod(SparkMaxPeriodicFrameType.Status0, TuningConstants.FLYWHEEL_SENSOR_FRAME_PERIOD_MS);
        this.flywheelMotor.setSelectedSlot(DefaultPidSlotId);
        
        this.flywheelMotor.burnFlash();

        // FLYWHEEL FOLLOWER
        ISparkMax flywheelFollower = provider.getSparkMax(ElectronicsConstants.FLYWHEEL_FOLLOWER_MOTOR_CAN_ID, SparkMaxMotorType.Brushed);
        flywheelFollower.setInvertOutput(TuningConstants.SHOOTER_MOTOR_FOLLOWER_INVERT_OUTPUT);
        flywheelFollower.setNeutralMode(MotorNeutralMode.Coast);
        flywheelFollower.follow(flywheelMotor);
        flywheelFollower.setCurrentLimit(TuningConstants.FLYWHEEL_FOLLOWER_STALL_LIMIT, TuningConstants.FLYWHEEL_FOLLOWER_FREE_LIMIT, TuningConstants.FLYWHEEL_FOLLOWER_RPM_LIMIT);

        flywheelFollower.burnFlash();

        // THROUGH BEAM
        this.throughBeamSensor = provider.getAnalogInput(ElectronicsConstants.INTAKE_THROUGHBEAM_ANALOG_INPUT);

        this.useShootAnywayMode = false;
        this.outTakeStartTime = 0.0;
        this.shootingStartTime = 0.0;

        this.currentEffectorState = EffectorState.Off;
    }

    @Override
    public void readSensors()
    {
        this.intakeMotorVelocity = this.intakeMotor.getVelocity();

        this.flywheelPosition = this.flywheelMotor.getPosition();
        this.flywheelVelocity = this.flywheelMotor.getVelocity();
        this.flywheelError = this.flywheelMotor.getOutput() - this.flywheelSetpoint;

        this.logger.logNumber(LoggingKey.ShooterFlywheelPosition, this.flywheelPosition);
        this.logger.logNumber(LoggingKey.ShooterFlywheelVelocity, this.flywheelVelocity);
        this.logger.logNumber(LoggingKey.ShooterFlywheelError, this.flywheelError);

        this.throughBeamSensorValue = this.throughBeamSensor.getVoltage();
        this.throughBeamBroken = this.throughBeamSensorValue < TuningConstants.INTAKE_THROUGHBEAM_CUTOFF;

        this.logger.logNumber(LoggingKey.IntakeThroughBeamSensorValue, this.throughBeamSensorValue);
        this.logger.logBoolean(LoggingKey.IntakeThroughBeamBroken, this.throughBeamBroken);

        double batteryVoltage = this.powerManager.getBatteryVoltage();
        
        this.logger.logNumber(LoggingKey.IntakeMotorVelocity, this.intakeMotorVelocity);
    }

    @Override
    public void update()
    {
        double currTime = this.timer.get();

        double intakePower = TuningConstants.ZERO;

        // FLYWHEEL LOGIC
        double flywheelMotorPower = this.driver.getAnalog(AnalogOperation.FlywheelMotorPower);
        double flywheelVelocityGoal = this.driver.getAnalog(AnalogOperation.FlywheelVelocityGoal); // This value should be calculated and in RPM

        if (flywheelMotorPower != TuningConstants.MAGIC_NULL_VALUE)
        {
            this.flywheelSetpoint = this.flywheelVelocity;
            this.flywheelMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            this.flywheelMotor.set(flywheelMotorPower);
            this.logger.logNumber(LoggingKey.FlywheelPower, flywheelMotorPower);
        }
        else if (flywheelVelocityGoal != TuningConstants.MAGIC_NULL_VALUE)
        {
            this.flywheelSetpoint = flywheelVelocityGoal;
            this.flywheelMotor.setControlMode(SparkMaxControlMode.Velocity);
            this.flywheelMotor.set(this.flywheelSetpoint);
            this.logger.logNumber(LoggingKey.FlywheelPower, -1318.0);
        }
        else
        {
            this.flywheelSetpoint = TuningConstants.MAGIC_NULL_VALUE;
            this.flywheelMotor.stop();
            this.logger.logNumber(LoggingKey.FlywheelPower, TuningConstants.MAGIC_NULL_VALUE);
        }

        this.logger.logNumber(LoggingKey.FlywheelDesiredVelocity, this.flywheelSetpoint);

        // STATE SWITCHING

        // Make another case here and make sure we can leave shooting

        switch (this.currentEffectorState)
        {

            case Off:
                
                // Start intaking when told to
                if (this.driver.getDigital(DigitalOperation.IntakeIn))
                {
                    this.currentEffectorState = EffectorState.Intaking;
                }

                // Start shooting if told to, and flywheel is spun up or we don't care about spun up
                else if (this.driver.getDigital(DigitalOperation.FeedRing) && (isFlywheelSpunUp() || this.useShootAnywayMode) )
                {
                    this.currentEffectorState = EffectorState.Shooting;
                    this.shootingStartTime = currTime;
                }

                // Start outtaking when told to
                else if (this.driver.getDigital(DigitalOperation.IntakeOut))
                {
                    this.currentEffectorState = EffectorState.Outtaking;
                    this.outTakeStartTime = currTime;
                }
                break;

            case Intaking:

                // Stop if forced to
                if(this.driver.getDigital(DigitalOperation.ForceStop))
                {
                    this.currentEffectorState = EffectorState.Off;
                }

                // shoot if told to, and were not intaking 
                else if(this.driver.getDigital(DigitalOperation.FeedRing) && !this.driver.getDigital(DigitalOperation.IntakeIn) && (isFlywheelSpunUp() || this.useShootAnywayMode))
                {
                    this.currentEffectorState = EffectorState.Shooting;
                    this.shootingStartTime = currTime;
                }

                // outtake if told to
                else if(this.driver.getDigital(DigitalOperation.IntakeOut))
                {
                    this.currentEffectorState = EffectorState.Outtaking;
                    this.outTakeStartTime = currTime;
                }

                // if through beam broken, and we can stop intake when desired then stop
                else if(this.throughBeamBroken && !this.driver.getDigital(DigitalOperation.ForceIntake))
                {
                    this.currentEffectorState = EffectorState.Off;
                }
                break;

            case Shooting:

                // stop if forced to
                if(this.driver.getDigital(DigitalOperation.ForceStop))
                {
                    this.currentEffectorState = EffectorState.Off;
                }

                // if through beam is not broken, and we've passed expected shoot time
                else if(this.shootingStartTime + TuningConstants.EFFECTOR_SHOOTING_DURATION < currTime && !this.throughBeamBroken)
                {
                    this.currentEffectorState = EffectorState.Off;
                }
                break;
            
            case Outtaking:

                // stop if forced to
                if(this.driver.getDigital(DigitalOperation.ForceStop))
                {
                    this.currentEffectorState = EffectorState.Off;
                }

                // intake if told to
                else if(this.driver.getDigital(DigitalOperation.IntakeIn))
                {
                    this.currentEffectorState = EffectorState.Intaking;
                }

                // feed ring if told to
                else if( (isFlywheelSpunUp() || this.useShootAnywayMode) && this.driver.getDigital(DigitalOperation.FeedRing))
                {
                    this.currentEffectorState = EffectorState.Shooting;
                    this.shootingStartTime = currTime;
                }
                
                // Turn off if outtake time has passed and through beam is no longer broken
                else if(this.outTakeStartTime + TuningConstants.EFFECTOR_OUTTAKE_DURATION < currTime && !this.throughBeamBroken)
                {
                    this.currentEffectorState = EffectorState.Off;
                }
                break;            
        }

        // STATE CONTROL

        switch (this.currentEffectorState)
        {

            case Intaking:
                intakePower = TuningConstants.EFFECTOR_INTAKE_IN_POWER;
                this.intakeMotor.set(intakePower);
                break;
            
            case Outtaking:
                intakePower = TuningConstants.EFFECTOR_INTAKE_OUT_POWER;
                this.intakeMotor.set(intakePower);
                break;

            case Shooting:
                intakePower = TuningConstants.EFFECTOR_INTAKE_FEED_SHOOTER_POWER;
                this.intakeMotor.set(intakePower);
                break;
            
            case Off:
                this.intakeMotor.set(intakePower);
                break;
        }

        this.logger.logNumber(LoggingKey.IntakeMotorPercentOutput, intakePower);

        this.prevTime = currTime;
    }

    @Override
    public void stop()
    {
        this.intakeMotor.stop();
        this.flywheelMotor.stop();

        this.outTakeStartTime = 0.0;
    }

    public double getFlywheelSetpoint()
    {
        return this.flywheelSetpoint;
    }

    public boolean isFlywheelSpunUp()
    {
        return this.flywheelSetpoint > 0.0 && Math.abs(this.flywheelError) <= TuningConstants.FLYWHEEL_ALLOWABLE_ERROR_RANGE;
    }

    public boolean hasGamePiece()
    {
        return this.throughBeamBroken;
    }
}