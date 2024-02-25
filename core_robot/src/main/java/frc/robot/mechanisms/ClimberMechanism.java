package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.*;
import frc.robot.ElectronicsConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;

//Inject Functions
import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ClimberMechanism implements IMechanism
{
    private final ITalonSRX climberMotor;
    // private final IServo ratchetServo;
    private final IDigitalInput climberLimitSwitch;

    private final IDriver driver;
    private final ILogger logger;

    private boolean isClimberDown;
    // private double servoPos;

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

        // this.ratchetServo = provider.getServo(ElectronicsConstants.CLIMBER_SERVO_MOTOR_CAN_ID);
        this.climberLimitSwitch = provider.getDigitalInput(ElectronicsConstants.CLIMBER_LIMIT_SWITCH_DIO_CHANNEL);
        // this.servoPos = TuningConstants.CLIMBER_SERVO_DOWN_POSITION;
    }

    @Override
    public void readSensors()
    {
        this.isClimberDown = !this.climberLimitSwitch.get();
        this.logger.logBoolean(LoggingKey.ClimberLimitSwitch, this.isClimberDown);
    }

    @Override
    public void update()
    {
        // press and hold button for climber to go down / up
        double climberPowerAdjustment = TuningConstants.ZERO;
        if (this.driver.getDigital(DigitalOperation.ClimberWinchDown))
        {
            climberPowerAdjustment = TuningConstants.CLIMBER_WINCH_DOWN_POWER;
        }
        // else if (this.driver.getDigital(DigitalOperation.ClimberWinchDown))
        // {
        //     climberPowerAdjustment = TuningConstants.CLIMBER_WINCH_DOWN_POWER;
        // }

        this.climberMotor.set(climberPowerAdjustment);
        this.logger.logNumber(LoggingKey.ClimberMotorPower, climberPowerAdjustment);

        // press button for servo to go down / up
        // if (this.driver.getDigital(DigitalOperation.ClimberServoUp))
        // {
        //     this.servoPos = TuningConstants.CLIMBER_SERVO_UP_POSITION;
        // }
        // else if (this.driver.getDigital(DigitalOperation.ClimberServoDown))
        // {
        //     this.servoPos = TuningConstants.CLIMBER_SERVO_DOWN_POSITION;
        // }

        // this.ratchetServo.set(servoPos);
        // this.logger.logNumber(LoggingKey.ClimberServoPosition, servoPos);
    }

    @Override
    public void stop()
    {
      this.climberMotor.stop();
    }

    public boolean getClimberDown()
    {
        return this.isClimberDown;
    }
}
