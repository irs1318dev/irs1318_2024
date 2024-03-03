package frc.robot.driver.controltasks;

import frc.robot.driver.AnalogOperation;

public class ShooterSpinTask extends TimedTask
{
    private final double flywheelSpeed;

    public ShooterSpinTask(double flywheelSpeed)
    {
        this(flywheelSpeed, 60.0);
    }

    public ShooterSpinTask(double flywheelSpeed, double duration)
    {
        super(duration);

        this.flywheelSpeed = flywheelSpeed;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, this.flywheelSpeed);
        this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, this.flywheelSpeed);
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.EndEffectorNearFlywheelVelocityGoal, this.flywheelSpeed);
        this.setAnalogOperationState(AnalogOperation.EndEffectorFarFlywheelVelocityGoal, this.flywheelSpeed);
    }
}
