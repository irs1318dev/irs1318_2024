package frc.lib.robotprovider;

import javax.inject.Inject;
import javax.inject.Singleton;

import org.littletonrobotics.junction.Logger;

import frc.robot.LoggingKey;

/**
 * Logger that logs current values to a dashboard.
 *
 */
@Singleton
public class AdvantageKitLogger implements ISmartDashboardLogger
{
    @Inject
    public AdvantageKitLogger()
    {
    }

    /**
     * Write a boolean to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logBoolean(LoggingKey key, boolean value)
    {
        Logger.recordOutput(key.value, value);
    }

    /**
     * Write a boolean array to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logBooleanArray(LoggingKey key, boolean[] value)
    {
        Logger.recordOutput(key.value, value);
    }

    /**
     * Write a number (double) to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logNumber(LoggingKey key, double value)
    {
        Logger.recordOutput(key.value, value);
    }

    /**
     * Write a number (double) to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logNumber(LoggingKey key, Double value)
    {
        String valueString = "N/A";
        if (value != null)
        {
            valueString = String.valueOf(value);
        }

        this.logString(key, valueString);
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logInteger(LoggingKey key, int value)
    {
        this.logInteger(key, value, null);
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logInteger(LoggingKey key, Integer value)
    {
        String valueString = "N/A";
        if (value != null)
        {
            valueString = String.valueOf(value);
        }

        this.logString(key, valueString);
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param key to write to
     * @param value to write
     * @param formatString to use
     */
    @Override
    public void logInteger(LoggingKey key, int value, String formatString)
    {
        Logger.recordOutput(key.value, value);
    }

    /**
     * Write a string to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logString(LoggingKey key, String value)
    {
        if (value == null)
        {
            value = "";
        }

        Logger.recordOutput(key.value, value);
    }

    /**
     * Update the log, if appropriate..
     */
    @Override
    public void update()
    {
    }

    /**
     * Flush the output stream, if appropriate..
     */
    @Override
    public void flush()
    {
    }
}
