package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.PIDHandler;
import org.usfirst.frc.team1318.robot.driver.IControlTask;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.vision.VisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 * 
 * @author William
 */
public class VisionCenteringTask extends MoveDistanceTaskBase implements IControlTask
{
    public enum CenteringMode
    {
        Single, Continual, PID;
    }

    private final VisionManager visionManager;
    private final CenteringMode mode;

    private PIDHandler pidHandler;

    /**
    * Initializes a new VisionCenteringTask
    */
    public VisionCenteringTask()
    {
        this(CenteringMode.Single);
    }

    /**
    * Initializes a new VisionCenteringTask
    * @param mode to use for centering
    */
    public VisionCenteringTask(CenteringMode mode)
    {
        super(true);

        this.visionManager = this.getInjector().getInstance(VisionManager.class);
        this.mode = mode;

        if (this.mode == CenteringMode.PID)
        {
            this.pidHandler = new PIDHandler(0.02, 0.0, 0.0, 0.0, -0.2, 0.2);
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        System.out.println(String.format("%f centerAngle", this.visionManager.getCenter1Angle()));
        if (this.mode == CenteringMode.Single)
        {
            super.update();
        }
        else if (this.mode == CenteringMode.Continual)
        {
            // for continual calculations, reset the start encoder distance and recalculate the desired final encoder distances
            this.setStartEncoderDistance();
            this.determineFinalEncoderDistance();

            super.update();
        }
        else
        {
            this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);

            this.setAnalogOperationState(
                Operation.DriveTrainTurn,
                this.pidHandler.calculatePosition(0.0, this.visionManager.getCenter1Angle()));
        }
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        if (this.mode == CenteringMode.Single ||
            this.mode == CenteringMode.Continual)
        {
            super.stop();
        }
        else
        {
            this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
            this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        if (this.mode == CenteringMode.Single ||
            this.mode == CenteringMode.Continual)
        {
            super.end();
        }
        else
        {
            this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
            this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
        }
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        if (this.mode == CenteringMode.Single ||
            this.mode == CenteringMode.Continual)
        {
            return super.hasCompleted();
        }
        else
        {
            return Math.abs(this.visionManager.getCenter1Angle()) < TuningConstants.MAX_VISION_CENTERING_RANGE_DEGREES;
        }
    }

    /**
     * Determine the final encoder distance based off center in pixels
     */
    @Override
    protected void determineFinalEncoderDistance()
    {
        // Convert center in pixels to degrees with 0 degrees being desired place
        Double centerAngle = this.visionManager.getCenter1Angle();
        if (centerAngle != null)
        {
            // Set desired encoder distances based on degrees off of center
            double arcLength = Math.PI * HardwareConstants.DRIVETRAIN_WHEEL_SEPARATION_DISTANCE * (centerAngle / 360.0);
            this.desiredFinalLeftEncoderDistance = this.startLeftEncoderDistance + arcLength;
            this.desiredFinalRightEncoderDistance = this.startLeftEncoderDistance - arcLength;
        }
        else
        {
            this.desiredFinalLeftEncoderDistance = this.startLeftEncoderDistance;
            this.desiredFinalRightEncoderDistance = this.startLeftEncoderDistance;
        }
    }
}
