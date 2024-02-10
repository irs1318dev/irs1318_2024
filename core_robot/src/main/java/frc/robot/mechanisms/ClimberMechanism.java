package frc.robot.mechanisms;

import frc.lib.controllers.TrapezoidProfile;
import frc.lib.driver.IDriver;
import frc.lib.filters.FloatingAverageCalculator;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.*;
import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
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

    private final IDriver driver;

    private final ISparkMax climberMotor;
    private final ISparkMax followClimberMotor;

    private final ILogger logger;
    
    private final IServo servo;
    
    @Inject
    public ClimberMechanism(IRobotProvider provider, IDriver driver, LoggingManager logger)
    {
        this.driver = driver;

        this.climberMotor = provider.getSparkMax(ElectronicsConstants.CLIMBER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.climberMotor.setControlMode(SparkMaxControlMode.PercentOutput);
        this.climberMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.climberMotor.burnFlash();
        this.climberMotor.setInvertOutput(false);

        this.followClimberMotor = provider.getSparkMax(ElectronicsConstants.FOLLOWER_CLIMBER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.followClimberMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.followClimberMotor.burnFlash();
        this.followClimberMotor.setInvertOutput(true);
        this.followClimberMotor.follow(this.climberMotor);

        this.servo = provider.getServo();
        this.logger = logger;

        

    }

    @Override
    public void readSensors()
    {
        
    }

    @Override
    public void update()
    {
        double climberPowerAdjustment = this.driver.getAnalog(AnalogOperation.ClimberShoulderPower);
        
        this.climberMotor.set(climberPowerAdjustment);

        if (this.driver.getDigital(DigitalOperation.ServoUp)) 
        {
            this.servo.set(1.0);
        }
        else if (this.driver.getDigital(DigitalOperation.ServoDown)) 
        {
            this.servo.set(-1.0);
        }

    }

    @Override
    public void stop()
    {
        this.climberMotor.stop();
    }
    
}
