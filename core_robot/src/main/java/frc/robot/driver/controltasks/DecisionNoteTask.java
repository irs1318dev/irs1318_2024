package frc.robot.driver.controltasks;

import frc.lib.driver.IControlTask;
import frc.robot.mechanisms.EndEffectorMechanism;

public class DecisionNoteTask extends DecisionSequentialTask
{
    private final IControlTask hasNoteTask;
    private final IControlTask noNoteTask;

    public DecisionNoteTask(IControlTask hasNodeTask, IControlTask noNoteTask)
    {
        super();
        this.hasNoteTask = hasNodeTask;
        this.noNoteTask = noNoteTask;
    }

    public void begin()
    {
        super.begin();
        EndEffectorMechanism eeMechanism = this.getInjector().getInstance(EndEffectorMechanism.class);
        if (eeMechanism.hasGamePiece())
        {
            System.out.println("has gamepiece");
            this.AppendTask(this.hasNoteTask);
        }
        else
        {
            System.out.println("no gamepiece");
            this.AppendTask(this.noNoteTask);
        }
    }
}
