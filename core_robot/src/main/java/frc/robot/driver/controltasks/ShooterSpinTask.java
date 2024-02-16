package frc.robot.driver.controltasks;

import frc.robot.driver.AnalogOperation;

public class ShooterSpinTask extends TimedTask
{
    private final double flywheelSpeed;

    public ShooterSpinTask(double flywheelSpeed)
    {
        this(flywheelSpeed, 0.5);
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

        this.setAnalogOperationState(AnalogOperation.NearFlywheelVelocityGoal, this.flywheelSpeed);
        this.setAnalogOperationState(AnalogOperation.FarFlywheelVelocityGoal, this.flywheelSpeed);
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.NearFlywheelVelocityGoal, this.flywheelSpeed);
        this.setAnalogOperationState(AnalogOperation.FarFlywheelVelocityGoal, this.flywheelSpeed);
    }
}
