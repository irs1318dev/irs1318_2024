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
        double climberPowerAdjustment = this.driver.getAnalog(AnalogOperation.ClimberPower);
        double servoPos = TuningConstants.MAGIC_NULL_VALUE;
        
        this.climberMotor.set(climberPowerAdjustment);


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

        this.logger.logNumber(LoggingKey.ClimberMotorPower, climberPowerAdjustment);
        this.logger.logNumber(LoggingKey.ClimberServoPosition, servoPos);
    }

    @Override
    public void stop()
    {
        this.climberMotor.stop();
    }
}
