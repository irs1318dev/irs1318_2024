package frc.lib.robotprovider;

import frc.robot.IRealWorldSimulator;
import frc.lib.controllers.PIDHandler;

public class FauxbotTalonFX extends FauxbotAdvancedMotorBase implements ITalonFX
{
    private final IRealWorldSimulator simulator;

    private FauxbotEncoder innerEncoder;
    private PIDHandler pidHandler;

    private TalonFXControlMode currentMode;
    private double kp;
    private double ki;
    private double kd;
    private double kf;

    public FauxbotTalonFX(int deviceNumber, IRealWorldSimulator simulator)
    {
        super(deviceNumber);

        this.simulator = simulator;
        this.currentMode = TalonFXControlMode.PercentOutput;

        this.innerEncoder = new FauxbotEncoder(new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, FauxbotTalonFX.class, this.connection.getPort()));
    }

    public void follow(ITalonFX talonFX)
    {
    }

    public void follow(ITalonFX talonFX, boolean invertDirection)
    {
    }

    public void setControlMode(TalonFXControlMode mode)
    {
        this.currentMode = mode;
        this.resetPID();
    }

    @Override
    public void clearRemoteSensor()
    {
    }

    @Override
    public void setRemoteSensor(int sensorId, double ratio)
    {
    }

    @Override
    public void setFeedbackUpdateRate(double frequencyHz)
    {
    }

    @Override
    public void setErrorUpdateRate(double frequencyHz)
    {
    }

    @Override
    public void updateLimitSwitchConfig(boolean forwardEnabled, boolean forwardNormallyOpen, boolean forwardReset, double forwardResetPosition, boolean reverseEnabled, boolean reverseNormallyOpen, boolean reverseReset, double reverseResetPosition)
    {
    }

    @Override
    public void setSupplyCurrentLimit(boolean enabled, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
    {
    }

    public void setSelectedSlot(int slotId)
    {
    }

    public void setPIDF(double p, double i, double d, double f, int slotId)
    {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = f;
        this.resetPID();
    }

    public void setMotionMagicPIDVS(double p, double i, double d, double v, double s, double cruiseVelocity, double maxAcceleration, double maxJerk, int slotId)
    {
    }

    @Override
    public void set(double newValue)
    {
        this.set(this.currentMode, newValue);
    }

    @Override
    public void set(TalonFXControlMode mode, double newValue)
    {
        this.set(this.currentMode, newValue);
    }

    @Override
    public void set(TalonFXControlMode mode, int slotId, double newValue)
    {
        if (mode == TalonFXControlMode.Follower)
        {
            FauxbotActuatorBase actuator = FauxbotActuatorManager.get(new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, (int)newValue));
            if (actuator != null && actuator instanceof FauxbotAdvancedMotorBase)
            {
                FauxbotAdvancedMotorBase advancedMotor = (FauxbotAdvancedMotorBase)actuator;
                advancedMotor.currentPowerProperty.addListener(
                    (observable, oldValue, value) -> { this.currentPowerProperty.set((Double)value); });
            }
            else
            {
                throw new RuntimeException("expected a different actuator type " + actuator == null ? "null" : actuator.toString());
            }
        }
        else if (mode == TalonFXControlMode.Velocity && this.pidHandler != null)
        {
            super.set(this.pidHandler.calculateVelocity(newValue, innerEncoder.getRate()));
        }
        else if (mode == TalonFXControlMode.Position && this.pidHandler != null)
        {
            super.set(this.pidHandler.calculatePosition(newValue, innerEncoder.get()));
        }
        else
        {
            super.set(newValue);
        }
    }

    public void updateLimitSwitchConfig(boolean forwardEnabled, boolean forwardNormallyOpen, boolean reverseEnabled, boolean reverseNormallyOpen)
    {
    }

    public void setMotorOutputSettings(boolean invert, MotorNeutralMode neutralMode)
    {
    }

    public void setInvertSensor(boolean flip)
    {
    }

    public void setNeutralMode(MotorNeutralMode neutralMode)
    {
    }

    public void setVoltageCompensation(boolean enabled, double maxVoltage)
    {
    }

    public void stop()
    {
    }

    public void setPosition(double position)
    {
    }

    public void reset()
    {
    }

    public double getPosition()
    {
        return this.innerEncoder.getDistance();
    }

    public double getVelocity()
    {
        return 0.0;
    }

    public double getError()
    {
        return 0.0;
    }

    public TalonXLimitSwitchStatus getLimitSwitchStatus()
    {
        return new TalonXLimitSwitchStatus(false, false);
    }

    private void resetPID()
    {
        if (this.simulator.shouldSimulatePID() &&
            (this.currentMode == TalonFXControlMode.Position || this.currentMode == TalonFXControlMode.Velocity))
        {
            ITimer timer = new FauxbotTimer();
            timer.start();
            this.pidHandler = new PIDHandler(this.kp, this.ki, this.kd, this.kf, 1.0, -4096.0, 4096.0, timer);
        }
        else
        {
            this.pidHandler = null;
        }
    }
}
