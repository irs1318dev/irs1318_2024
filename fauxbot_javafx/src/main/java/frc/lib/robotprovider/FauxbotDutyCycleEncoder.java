package frc.lib.robotprovider;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;

public class FauxbotDutyCycleEncoder extends FauxbotSensorBase implements IDutyCycleEncoder
{
    private final DoubleProperty dutyCycleEncoderProperty;
    private double distancePerRotation;

    public FauxbotDutyCycleEncoder(int port)
    {
        this.dutyCycleEncoderProperty = new SimpleDoubleProperty();
        FauxbotSensorManager.set(new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.DigitalInput, this.getClass(), port), this);
    }

    public double get()
    {
        return this.dutyCycleEncoderProperty.getValue();
    }

    public double getDistance()
    {
        return this.get() * this.distancePerRotation;
    }

    public double getAbsolutePosition()
    {
        return 0.0;
    }

    public int getFrequency()
    {
        return 0;
    }

    public boolean isConnected()
    {
        return true;
    }

    public void setConnectedFrequencyThreshold(int frequency)
    {
    }

    public void setDistancePerRotation(double distancePerRotation)
    {
        this.distancePerRotation = distancePerRotation;
    }

    public void setDutyCycleRange(double min, double max)
    {
    }

    public void setPositionOffset(double offset)
    {
    }

    public void reset()
    {
    }


    public void set(double newValue)
    {
        this.dutyCycleEncoderProperty.setValue(newValue);
    }

    public DoubleProperty getProperty()
    {
        return this.dutyCycleEncoderProperty;
    }
}
