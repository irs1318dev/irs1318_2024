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
    private final IDigitalInput climberLimitSwitch;

    private final IDriver driver;
    private final ILogger logger;

    private boolean isClimberDown;

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

        this.climberLimitSwitch = provider.getDigitalInput(ElectronicsConstants.CLIMBER_LIMIT_SWITCH_DIO_CHANNEL);
    }

    @Override
    public void readSensors()
    {
        this.isClimberDown = !this.climberLimitSwitch.get();
        this.logger.logBoolean(LoggingKey.ClimberLimitSwitch, this.isClimberDown);
    }

    @Override
    public void update(RobotMode mode)
    {
        // press and hold button for climber to go down / up
        double climberPowerAdjustment = TuningConstants.ZERO;
        if (this.driver.getDigital(DigitalOperation.ClimberWinchDown))
        {
            climberPowerAdjustment = TuningConstants.CLIMBER_WINCH_DOWN_POWER;
        }
        
        if (TuningConstants.USE_CLIMBER_LIMIT_SWITCH && this.isClimberDown)
        {
            this.climberMotor.set(TuningConstants.ZERO);
        }
        else
        {
            
        this.climberMotor.set(climberPowerAdjustment);
        }

        this.logger.logNumber(LoggingKey.ClimberMotorPower, climberPowerAdjustment);
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
