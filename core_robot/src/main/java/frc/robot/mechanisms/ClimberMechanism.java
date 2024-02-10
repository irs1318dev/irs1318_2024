package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.*;
import frc.robot.ElectronicsConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;

//Inject Functions
import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ClimberMechanism implements IMechanism 
{
    private final ISparkMax climberMotor;
    private final IServo servo;

    private final IDriver driver;
    private final ILogger logger;

    @Inject
    public ClimberMechanism(IRobotProvider provider, IDriver driver, LoggingManager logger)
    {
        this.driver = driver;
        this.logger = logger;

        this.climberMotor = provider.getSparkMax(ElectronicsConstants.CLIMBER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.climberMotor.setControlMode(SparkMaxControlMode.PercentOutput);
        this.climberMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.climberMotor.setInvertOutput(TuningConstants.CLIMBER_MOTOR_INVERT_OUTPUT);
        this.climberMotor.burnFlash();

        ISparkMax followClimberMotor = provider.getSparkMax(ElectronicsConstants.CLIMBER_MOTOR_FOLLOWER_CAN_ID, SparkMaxMotorType.Brushless);
        followClimberMotor.setNeutralMode(MotorNeutralMode.Brake);
        followClimberMotor.setInvertOutput(TuningConstants.CLIMBER_MOTOR_FOLLOWER_INVERT_OUTPUT);
        followClimberMotor.follow(this.climberMotor);
        followClimberMotor.burnFlash();

        this.servo = provider.getServo(ElectronicsConstants.CLIMBER_SERVO_MOTOR_CAN_ID);
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update()
    {
        double climberPowerAdjustment = this.driver.getAnalog(AnalogOperation.ClimberShoulderPower);
        this.logger.logNumber(LoggingKey.ClimberMotorPower, climberPowerAdjustment);
        this.climberMotor.set(climberPowerAdjustment);

        if (this.driver.getDigital(DigitalOperation.ServoUp))
        {
            double climberServo = TuningConstants.CLIMBER_SERVO_UP_POWER;
            this.servo.set(climberServo);
            this.logger.logNumber(LoggingKey.ClimberServoPower, climberServo);
        }
        else if (this.driver.getDigital(DigitalOperation.ServoDown))
        {
            double climberServo = TuningConstants.CLIMBER_SERVO_DOWN_POWER; 
            this.servo.set(climberServo);
            this.logger.logNumber(LoggingKey.ClimberServoPower, climberServo);
        }
    }

    @Override
    public void stop()
    {
        this.climberMotor.stop();
    }
}
