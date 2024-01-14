package frc.robot.mechanisms;

import frc.lib.mechanisms.*;
import frc.lib.robotprovider.ITalonFX;
import frc.lib.robotprovider.ITalonSRX;
import frc.lib.robotprovider.TalonFXControlMode;
import frc.robot.driver.DigitalOperation;
import src.main.java.frc.lib.driver.IDriver;
import src.main.java.frc.lib.mechanisms.IMechanism;
import src.main.java.frc.lib.robotprovider.ILogger;
import src.main.java.frc.lib.robotprovider.IRobotProvider;
import src.main.java.frc.lib.robotprovider.ITimer;
import src.main.java.frc.robot.LoggingKey;
import src.main.java.frc.robot.TuningConstants;

//Inject Functions
import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class ChainArmMechanism implements IMechanism{

    //----------------- General variables -----------------//

    private static final int defaultPidSlotId = 0;

    private final IDriver driver;
    private final ILogger logger;
    private double Position;
    private final PowerManager powerManager;
    private final double leftMotorPosition;
    private final double leftMotorVelocity;
    private double prevTime;
    private final ITalonSRX rightMotor;
    private final ITalonSRX leftMotor;

    @Inject
    public ChainArmMechanism(IRobotProvider provider, IDriver driver)
    {
        this.driver = driver;
        this.rightMotor = provider.getTalonSRX(0);
        this.leftMotor = provider.getTalonSRX(1);
        this.leftMotor.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.leftMotor.setInvertOutput(true); // create a constant
        this.leftMotor.setPosition(0.0);
        this.leftMotor.setPIDF(0.0, 0.0, 0.0, 0.0);
        this.rightMotor.follow(this.leftMotor);
    }
    
    @Override
    public void readSensors()
    {
        this.leftMotorPosition = leftMotor.Position;
        this.leftMotorVelocity = leftMotor.Velocity;
        this.logger.logNumber(LoggingKey.LeftMotorPosition, this.leftMotorPosition);
        this.logger.logNumber(LoggingKey.LeftMotorVelocity, this.leftMotorVelocity);
    }

    @Override
    public void update()
    {
        if(DigitalOperation.ArmUsePID)
        {
            this.leftMotor.setControlMode(TalonXControlMode.Position);
            this.leftMotor.setPosition(this.driver.getAnalog(AnalogOperation.ChainArm));
        }
        
        else if(DigitalOperation.ArmUsePercentOutput)
        {
            this.leftMotor.setControlMode(TalonXControlMode.PercentOutput);
            this.leftMotor.set(this.driver.getAnalog(AnalogOperation.ChainArm));
        }
        
    }

    @Override
    public void stop()
    {
        this.leftMotor.setControlMode(TalonXControlMode.PercentOutput);
        this.rightMotor.setControlMode(TalonXControlMode.PercentOutput);

        this.leftMotor.set(0.0);
        this.rightMotor.set(0.0);
    }
}
