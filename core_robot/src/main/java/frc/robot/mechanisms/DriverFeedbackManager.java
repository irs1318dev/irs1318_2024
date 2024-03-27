package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.IMechanism;
import frc.lib.robotprovider.*;
import frc.robot.TuningConstants;
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
    private final ITimer timer;
    private final EndEffectorMechanism endEffector;
    
    private final PowerManager powerMan;

    private enum IntakeState
    {
        HasNoteRumble,
        HasNoteRumbleOver,
        NoNote,
    }

    private IntakeState intakeState;
    public Boolean rumble;
    private double intakeStateChangeTime;

    @Inject
    public DriverFeedbackManager(
        IDriver driver,
        IRobotProvider provider,
        ITimer timer,
        PowerManager powerMan,
        EndEffectorMechanism endEffector)
    {
        this.driver = driver;
        this.timer = timer;
        this.endEffector = endEffector;
        this.intakeState = IntakeState.HasNoteRumbleOver;

        this.ds = provider.getDriverStation();
        this.powerMan = powerMan;
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update(RobotMode mode)
    {
        RobotMode currentMode = this.ds.getMode();
        boolean isCurrentLimiting = this.powerMan.getCurrentLimitingValue() != CurrentLimiting.Normal;

        if (mode == RobotMode.Teleop || mode == RobotMode.Test)
        {
            double now = this.timer.get();
            switch (this.intakeState)
            {
                case HasNoteRumble:
                    if (!this.endEffector.hasGamePiece())
                    {
                        this.intakeState = IntakeState.NoNote;
                        this.intakeStateChangeTime = now;
                    }
                    else if (this.intakeStateChangeTime + TuningConstants.INTAKE_HAS_NOTE_RUMBLE_DURATION < now)
                    {
                        this.intakeState = IntakeState.HasNoteRumbleOver;
                        this.intakeStateChangeTime = now;
                    }

                    break;

                case HasNoteRumbleOver:
                    if (!this.endEffector.hasGamePiece())
                    {
                        this.intakeState = IntakeState.NoNote;
                        this.intakeStateChangeTime = now;
                    }

                    break;

                case NoNote:
                    if (this.endEffector.hasGamePiece())
                    {
                        this.intakeState = IntakeState.HasNoteRumble;
                        this.intakeStateChangeTime = now;
                    }

                    break;
            }
        }

        if(rumble)
        {
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.5);
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.5);
        }

        if (this.driver.getDigital(DigitalOperation.ForceLightDriverRumble))
        {
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.25);
            this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.25);
        }
        else
        {
            switch (this.intakeState)
            {
                
                case HasNoteRumble:
                    this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.5);
                    this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.5);

                    this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Left, 0.5);
                    this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Right, 0.5);
                    break;

                case HasNoteRumbleOver:
                case NoNote:
                    this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.0);
                    this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);

                    this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Left, 0.0);
                    this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Right, 0.0);
                
                    break;

            }
        }
    }

    @Override
    public void stop()
    {
        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.0);
        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);
    }
}
