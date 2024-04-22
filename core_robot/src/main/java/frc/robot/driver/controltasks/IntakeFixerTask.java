package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.EndEffectorMechanism;

public class IntakeFixerTask extends TimedTask
{
    private EndEffectorMechanism endEffector;

    public IntakeFixerTask()
    {
        this(0.1);
    }

    public IntakeFixerTask(double duration)
    {
        super(duration);
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
