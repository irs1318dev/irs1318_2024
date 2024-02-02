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

        // NEAR FLYWHEEL MOTOR
        this.nearFlywheelMotor = provider.getSparkMax(ElectronicsConstants.NEAR_FLYWHEEL_MOTOR_CAN_ID, SparkMaxMotorType.Brushed);
        // this.nearFlywheelMotor.setRelativeEncoder();
        // this.nearFlywheelMotor.setInvertSensor(TuningConstants.NEAR_SHOOTER_MOTOR_INVERT_SENSOR);
        this.nearFlywheelMotor.setInvertOutput(TuningConstants.NEAR_SHOOTER_MOTOR_INVERT_OUTPUT);
        this.nearFlywheelMotor.setNeutralMode(MotorNeutralMode.Coast);

        this.nearFlywheelMotor.setPIDF(
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KP, 
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KI, 
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KD, 
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KF, 
            DefaultPidSlotId);

        this.nearFlywheelMotor.setVelocityConversionFactor(TuningConstants.SHOOTER_FLYWHEEL_CONVERSION_FACTOR);
        this.nearFlywheelMotor.setCurrentLimit(TuningConstants.FLYWHEEL_STALL_LIMIT, TuningConstants.FLYWHEEL_FREE_LIMIT, TuningConstants.FLYWHEEL_RPM_LIMIT);
        this.nearFlywheelMotor.setFeedbackFramePeriod(SparkMaxPeriodicFrameType.Status0, TuningConstants.FLYWHEEL_SENSOR_FRAME_PERIOD_MS);
        this.nearFlywheelMotor.setSelectedSlot(DefaultPidSlotId);
        
        this.nearFlywheelMotor.burnFlash();

        // FAR FLYWHEEL MOTOR
        this.farFlywheelMotor = provider.getSparkMax(ElectronicsConstants.FAR_FLYWHEEL_MOTOR_CAN_ID, SparkMaxMotorType.Brushed);
        // this.farFlywheelMotor.setRelativeEncoder();
        // this.farFlywheelMotor.setInvertSensor(TuningConstants.FAR_SHOOTER_MOTOR_INVERT_SENSOR);
        this.farFlywheelMotor.setInvertOutput(TuningConstants.FAR_SHOOTER_MOTOR_INVERT_OUTPUT);
        this.farFlywheelMotor.setNeutralMode(MotorNeutralMode.Coast);

        this.farFlywheelMotor.setPIDF(
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KP, 
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KI, 
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KD, 
            TuningConstants.SHOOTER_FLYWHEEL_MOTOR_PID_KF, 
            DefaultPidSlotId);

        this.farFlywheelMotor.setVelocityConversionFactor(TuningConstants.SHOOTER_FLYWHEEL_CONVERSION_FACTOR);
        this.farFlywheelMotor.setCurrentLimit(TuningConstants.FLYWHEEL_STALL_LIMIT, TuningConstants.FLYWHEEL_FREE_LIMIT, TuningConstants.FLYWHEEL_RPM_LIMIT);
        this.farFlywheelMotor.setFeedbackFramePeriod(SparkMaxPeriodicFrameType.Status0, TuningConstants.FLYWHEEL_SENSOR_FRAME_PERIOD_MS);
        this.farFlywheelMotor.setSelectedSlot(DefaultPidSlotId);
        
        this.farFlywheelMotor.burnFlash();

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

        this.nearFlywheelPosition = this.nearFlywheelMotor.getPosition();
        this.nearFlywheelVelocity = this.nearFlywheelMotor.getVelocity();
        this.nearFlywheelError = this.nearFlywheelMotor.getOutput() - this.nearFlywheelSetpoint;

        this.logger.logNumber(LoggingKey.NearShooterFlywheelPosition, this.nearFlywheelPosition);
        this.logger.logNumber(LoggingKey.NearShooterFlywheelVelocity, this.nearFlywheelVelocity);
        this.logger.logNumber(LoggingKey.NearShooterFlywheelError, this.nearFlywheelError);

        this.farFlywheelPosition = this.farFlywheelMotor.getPosition();
        this.farFlywheelVelocity = this.farFlywheelMotor.getVelocity();
        this.farFlywheelError = this.farFlywheelMotor.getOutput() - this.farFlywheelSetpoint;

        this.logger.logNumber(LoggingKey.FarShooterFlywheelPosition, this.farFlywheelPosition);
        this.logger.logNumber(LoggingKey.FarShooterFlywheelVelocity, this.farFlywheelVelocity);
        this.logger.logNumber(LoggingKey.FarShooterFlywheelError, this.farFlywheelError);

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
        double nearFlywheelVelocityGoal = this.driver.getAnalog(AnalogOperation.NearFlywheelVelocityGoal); // This value should be calculated and in RPM
        double farFlywheelVelocityGoal = this.driver.getAnalog(AnalogOperation.FarFlywheelVelocityGoal); // This value should be calculated and in RPM
        

        if (flywheelMotorPower != TuningConstants.MAGIC_NULL_VALUE)
        {
            this.nearFlywheelSetpoint = this.nearFlywheelVelocity;
            this.farFlywheelSetpoint = this.farFlywheelVelocity;

            this.nearFlywheelMotor.setControlMode(SparkMaxControlMode.PercentOutput);
            this.farFlywheelMotor.setControlMode(SparkMaxControlMode.PercentOutput);

            this.nearFlywheelMotor.set(flywheelMotorPower);
            this.farFlywheelMotor.set(flywheelMotorPower);

            this.logger.logNumber(LoggingKey.FlywheelPower, flywheelMotorPower);
        }
        else if (nearFlywheelVelocityGoal != TuningConstants.MAGIC_NULL_VALUE && farFlywheelVelocityGoal != TuningConstants.MAGIC_NULL_VALUE)
        {
            this.nearFlywheelSetpoint = nearFlywheelVelocityGoal;
            this.farFlywheelSetpoint = farFlywheelVelocityGoal;

            this.nearFlywheelMotor.setControlMode(SparkMaxControlMode.Velocity);
            this.farFlywheelMotor.setControlMode(SparkMaxControlMode.Velocity);

            this.nearFlywheelMotor.set(this.nearFlywheelSetpoint);
            this.farFlywheelMotor.set(this.farFlywheelSetpoint);
            
            this.logger.logNumber(LoggingKey.FlywheelPower, -1318.0);
        }
        else
        {
            this.nearFlywheelSetpoint = TuningConstants.MAGIC_NULL_VALUE;
            this.farFlywheelSetpoint = TuningConstants.MAGIC_NULL_VALUE;
            
            this.nearFlywheelMotor.stop();
            this.farFlywheelMotor.stop();
            
            this.logger.logNumber(LoggingKey.FlywheelPower, TuningConstants.MAGIC_NULL_VALUE);
        }

        this.logger.logNumber(LoggingKey.NearFlywheelDesiredVelocity, this.nearFlywheelSetpoint);
        this.logger.logNumber(LoggingKey.FarFlywheelDesiredVelocity, this.farFlywheelSetpoint);
        

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
        this.nearFlywheelMotor.stop();
        this.farFlywheelMotor.stop();

        this.outTakeStartTime = 0.0;
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
        return (this.farFlywheelSetpoint > 0.0) &&
                (this.nearFlywheelSetpoint > 0.0) &&
                (Math.abs(this.farFlywheelError) <= TuningConstants.FLYWHEEL_ALLOWABLE_ERROR_RANGE) &&
                (Math.abs(this.nearFlywheelError) <= TuningConstants.FLYWHEEL_ALLOWABLE_ERROR_RANGE);
    }

    public boolean hasGamePiece()
    {
        return this.throughBeamBroken;
    }
}