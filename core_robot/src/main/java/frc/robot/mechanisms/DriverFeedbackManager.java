package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.IMechanism;
import frc.lib.robotprovider.*;
import frc.robot.driver.DigitalOperation;
import frc.lib.driver.descriptions.UserInputDevice;
import frc.robot.mechanisms.PowerManager.CurrentLimiting;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Driver feedback manager
 *
 * This class manages things like controller rumbler and indicator lights on the robot.
 *
 */
@Singleton
public class DriverFeedbackManager implements IMechanism
{
    private final IDriverStation ds;
    private final IDriver driver;
    
    private final PowerManager powerMan;

    @Inject
    public DriverFeedbackManager(
        IDriver driver,
        IRobotProvider provider,
        PowerManager powerMan)
    {
        this.driver = driver;

        this.ds = provider.getDriverStation();
        this.powerMan = powerMan;
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update()
    {
        RobotMode currentMode = this.ds.getMode();
        boolean isCurrentLimiting = this.powerMan.getCurrentLimitingValue() != CurrentLimiting.Normal;

        if (this.driver.getDigital(DigitalOperation.ForceLightDriverRumble))
        {
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.5);
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.5);
        }
        else
        {
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.0);
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);
        }
    }

    @Override
    public void stop()
    {
        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.0);
        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);
    }
}
