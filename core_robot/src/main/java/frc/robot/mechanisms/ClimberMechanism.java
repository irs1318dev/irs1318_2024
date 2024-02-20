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
    private final ITalonSRX climberMotor;
    private final IServo ratchetServo;

    private final IDriver driver;
    private final ILogger logger;

    @Inject
    public ClimberMechanism(IRobotProvider provider, IDriver driver, LoggingManager logger)
    {
        this.driver = driver;
        this.logger = logger;

        this.climberMotor = provider.getTalonSRX(ElectronicsConstants.CLIMBER_MOTOR_CAN_ID);
        this.climberMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        this.climberMotor.setMotorOutputSettings(TuningConstants.CLIMBER_MOTOR_INVERT_OUTPUT, MotorNeutralMode.Brake);

        ITalonSRX followClimberMotor = provider.getTalonSRX(ElectronicsConstants.CLIMBER_MOTOR_FOLLOWER_CAN_ID);
        followClimberMotor.setControlMode(TalonSRXControlMode.Follower);
        followClimberMotor.setMotorOutputSettings(TuningConstants.CLIMBER_MOTOR_FOLLOWER_INVERT_OUTPUT, MotorNeutralMode.Brake);
        followClimberMotor.follow(this.climberMotor);

        this.ratchetServo = provider.getServo(ElectronicsConstants.CLIMBER_SERVO_MOTOR_CAN_ID);
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update()
    {
        // press and hold button for climber to go down / up
        double climberPowerAdjustment = TuningConstants.MAGIC_NULL_VALUE;

        if (this.driver.getDigital(DigitalOperation.ClimberWinchUp))
        {
            climberPowerAdjustment = TuningConstants.CLIMBER_WINCH_UP_POWER;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberWinchDown))
        {
            climberPowerAdjustment = TuningConstants.CLIMBER_WINCH_DOWN_POWER;
        }

        // press button for servo to go down / up
        double servoPos = TuningConstants.MAGIC_NULL_VALUE;

        if (this.driver.getDigital(DigitalOperation.ClimberServoUp))
        {
            servoPos = TuningConstants.CLIMBER_SERVO_UP_POSITION;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberServoDown))
        {
            servoPos = TuningConstants.CLIMBER_SERVO_DOWN_POSITION;
        }


        if(servoPos != TuningConstants.MAGIC_NULL_VALUE)
        {
            this.ratchetServo.set(servoPos);
        }

        if(climberPowerAdjustment != TuningConstants.MAGIC_NULL_VALUE)
        {            
            this.climberMotor.set(climberPowerAdjustment);
        }

        this.logger.logNumber(LoggingKey.ClimberMotorPower, climberPowerAdjustment);
        this.logger.logNumber(LoggingKey.ClimberServoPosition, servoPos);
    }

    @Override
    public void stop()
    {
        this.climberMotor.stop();
    }
}
