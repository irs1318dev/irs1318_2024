package frc.team1318.robot.common.robotprovider;

public abstract class FauxbotSimpleMotorBase extends FauxbotMotorBase
{
    protected FauxbotActuatorConnection connection;

    protected FauxbotSimpleMotorBase(int port)
    {
        this.connection = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.PWM, port);
        FauxbotActuatorManager.set(this.connection, this);
    }
}
