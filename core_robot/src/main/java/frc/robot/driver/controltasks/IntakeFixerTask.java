package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.EndEffectorMechanism;

public class IntakeFixerTask extends TimedTask
{
    private EndEffectorMechanism endEffector;

    public IntakeFixerTask()
    {
        super(0.1);
    }

    @Override
    public void begin()
    {
        super.begin();
        this.endEffector = this.getInjector().getInstance(EndEffectorMechanism.class);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.IntakeOutSlow, true);
    }

    @Override
    public void end()
    {
        super.end();
        this.setDigitalOperationState(DigitalOperation.IntakeOutSlow, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return !this.endEffector.hasGamePiece() || super.hasCompleted();
    }
}
