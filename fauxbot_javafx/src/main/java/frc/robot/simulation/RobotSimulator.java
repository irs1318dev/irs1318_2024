package frc.robot.simulation;

import java.io.FileInputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.Calendar;

import frc.lib.robotprovider.*;
import frc.robot.IRealWorldSimulator;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;

@Singleton
public class RobotSimulator implements IRealWorldSimulator
{
    // private static final FauxbotSensorConnection EncoderAChannel = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.DigitalInput, FauxbotEncoder.class, 0);
    // private static final FauxbotSensorConnection EncoderBChannel = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.DigitalInput, FauxbotEncoder.class, 1);
    // private static final FauxbotActuatorConnection MotorChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.PWM, 0);

    private final FauxbotSensorConnection[] sensors =
        new FauxbotSensorConnection[]
        {
            // RobotSimulator.EncoderAChannel,
            // RobotSimulator.EncoderBChannel,
        };

    private final FauxbotActuatorConnection[] actuators =
        new FauxbotActuatorConnection[]
        {
            // RobotSimulator.MotorChannel,
        };

    @SuppressWarnings("serial")
    private final Map<FauxbotSensorConnection, String> sensorNameMap = new HashMap<FauxbotSensorConnection, String>()
    {
        {
            // this.put(RobotSimulator.EncoderAChannel, "Elevator encoder");
            // this.put(RobotSimulator.EncoderBChannel, "Elevator encoder");
        }
    };

    @SuppressWarnings("serial")
    private final Map<FauxbotActuatorConnection, String> motorNameMap = new HashMap<FauxbotActuatorConnection, String>()
    {
        {
            // this.put(RobotSimulator.MotorChannel, "Elevator motor");
        }
    };

    @Inject
    public RobotSimulator()
    {
    }

    @Override
    public FauxbotSensorConnection[] getSensors()
    {
        return this.sensors;
    }

    @Override
    public FauxbotActuatorConnection[] getActuators()
    {
        return this.actuators;
    }

    @Override
    public boolean getSensorTextBox(FauxbotSensorConnection connection)
    {
        return false;
    }

    @Override
    public String getSensorName(FauxbotSensorConnection connection)
    {
        if (this.sensorNameMap.containsKey(connection))
        {
            return this.sensorNameMap.get(connection);
        }

        return "Sensor " + connection;
    }

    @Override
    public double getSensorMin(FauxbotSensorConnection connection)
    {
        return -1.0;
    }

    @Override
    public double getSensorMax(FauxbotSensorConnection connection)
    {
        return 1.0;
    }

    @Override
    public String getActuatorName(FauxbotActuatorConnection connection)
    {
        if (this.motorNameMap.containsKey(connection))
        {
            return this.motorNameMap.get(connection);
        }

        return "Motor " + connection;
    }

    @Override
    public double getMotorMin(FauxbotActuatorConnection connection)
    {
        return -1.0;
    }

    @Override
    public double getMotorMax(FauxbotActuatorConnection connection)
    {
        return 1.0;
    }

    @Override
    public boolean shouldSimulatePID()
    {
        return true;
    }

    @Override
    public void update()
    {
        // double currTime = Calendar.getInstance().getTime().getTime() / 1000.0;
        // double currHeight = this.prevHeight;
        // double currVelocity = this.prevVelocity;

        // double motorPower = 0.0;
        // FauxbotActuatorBase actuator = FauxbotActuatorManager.get(RobotSimulator.MotorChannel);
        // if (actuator != null && actuator instanceof FauxbotMotorBase)
        // {
        //     FauxbotMotorBase motor = (FauxbotMotorBase)actuator;
        //     motorPower = motor.get();
        // }

        // double dt = currTime - this.prevTime;

        // currVelocity += motorPower * RobotSimulator.MotorStrength * dt + RobotSimulator.Gravity * dt;
        // currHeight += (currVelocity * dt);

        // if (currVelocity > RobotSimulator.ElevatorMaxVelocity)
        // {
        //     currVelocity = RobotSimulator.ElevatorMaxVelocity;
        // }
        // else if (currVelocity < RobotSimulator.ElevatorMinVelocity)
        // {
        //     currVelocity = RobotSimulator.ElevatorMinVelocity;
        // }

        // if (this.prevHeight > RobotSimulator.ElevatorMaxHeight)
        // {
        //     currHeight = RobotSimulator.ElevatorMaxHeight;
        //     currVelocity = 0.0;
        // }
        // else if (this.prevHeight < RobotSimulator.ElevatorMinHeight)
        // {
        //     currHeight = RobotSimulator.ElevatorMinHeight;
        //     currVelocity = 0.0;
        // }

        // this.prevHeight = currHeight;
        // this.prevTime = currTime;
        // this.prevVelocity = currVelocity;

        // FauxbotSensorBase sensor = FauxbotSensorManager.get(RobotSimulator.EncoderAChannel);
        // if (sensor != null && sensor instanceof FauxbotEncoder)
        // {
        //     FauxbotEncoder encoder = (FauxbotEncoder)sensor;
        //     encoder.set((int)this.prevHeight);
        // }
    }

    /**
     * Draw a frame of animation based on the current state of the simulation.
     * Remember that (0, 0) is at the top left!
     */
    @Override
    public void draw(Canvas canvas)
    {
        // double elevatorHeightRatio = this.prevHeight / (RobotSimulator.ElevatorMaxHeight - RobotSimulator.ElevatorMinHeight);

        // double canvasHeight = canvas.getHeight();
        // double canvasWidth = canvas.getWidth();
        // GraphicsContext gc = canvas.getGraphicsContext2D();
        // gc.clearRect(0.0, 0.0, canvasWidth, canvasHeight);

        // // cycle through the floors:
        // Color elevatorCarColor = Color.RED;
        // for (double ratio : RobotSimulator.FloorHeightPercentages)
        // {
        //     // if we are within a small allowance from the floor, change elevator color to Green
        //     if (Math.abs(elevatorHeightRatio - ratio) < RobotSimulator.PercentageAllowance)
        //     {
        //         elevatorCarColor = Color.GREEN;
        //     }

        //     // draw the floor:
        //     gc.setStroke(Color.BLACK);
        //     gc.setLineWidth(1.0);
        //     gc.strokeLine(RobotSimulator.ElevatorCarWidth, (1 - ratio) * canvasHeight, canvasWidth, (1 - ratio) * canvasHeight); 
        // }

        // // draw the elevator car:
        // gc.setStroke(elevatorCarColor);
        // gc.setLineWidth(1.0);
        // gc.strokeRect(
        //     0.0,
        //     (1.0 - elevatorHeightRatio) * canvasHeight - RobotSimulator.ElevatorCarHeight,
        //     RobotSimulator.ElevatorCarWidth,
        //     RobotSimulator.ElevatorCarHeight);

        // // draw the elevator rider:
        // gc.drawImage(this.elevatorPerson, 0.0, (1.0 - elevatorHeightRatio) * canvasHeight - RobotSimulator.ElevatorCarHeight, 
        //                 RobotSimulator.ElevatorCarWidth,  RobotSimulator.ElevatorCarHeight);
    }
}
