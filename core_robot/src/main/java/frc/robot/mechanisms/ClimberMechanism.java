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

    // do not know how to add a logger if we need it.
    
    @Inject
    public ClimberMechanism(IRobotProvider provider, IDriver driver)
    {
        this.driver = driver;

        this.climberMotor = provider.getSparkMax(ElectronicsConstants.CLIMBER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.followClimberMotor = provider.getSparkMax(ElectronicsConstants.FOLLOWER_CLIMBER_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);

        this.climberMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.followClimberMotor.setNeutralMode(MotorNeutralMode.Brake);

        this.climberMotor.burnFlash();
        this.followClimberMotor.burnFlash();

        followClimberMotor.follow(this.climberMotor);

        

    }

    @Override
    public void readSensors()
    {
        
    }

    @Override
    public void update()
    {
        double climberPowerAdjustment = this.driver.getAnalog(AnalogOperation.ClimberShoulderPower);

        double climberPower = 0.0;
        
        climberPower = climberPowerAdjustment;

    }

    @Override
    public void stop()
    {
        this.climberMotor.stop();
        this.followClimberMotor.stop();
    }
    
}
