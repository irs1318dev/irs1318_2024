package frc.lib.robotprovider;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;

public class FauxbotDigitalOutput extends FauxbotSensorBase implements IDigitalOutput
{
    private final BooleanProperty isSetProperty;

    public FauxbotDigitalOutput(int port)
    {
        this.isSetProperty = new SimpleBooleanProperty();
        FauxbotSensorManager.set(new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.DigitalInput, this.getClass(), port), this);
    }

    /**
     * gets the current value of the sensor
     * @return true if the sensor is set (closed), false otherwise (open)
     */
    public boolean get()
    {
        return this.isSetProperty.getValue();
    }

    public void set(boolean newValue)
    {
        this.isSetProperty.setValue(newValue);
    }

    public BooleanProperty getProperty()
    {
        return this.isSetProperty;
    }
}