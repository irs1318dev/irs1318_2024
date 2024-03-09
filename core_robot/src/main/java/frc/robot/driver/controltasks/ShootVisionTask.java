package frc.robot.driver.controltasks;

import frc.lib.helpers.LinearInterpolator;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ArmMechanism;
import frc.robot.mechanisms.OffboardVisionManager;

public class ShootVisionTask extends DecisionSequentialTask {
    private OffboardVisionManager vision;
    private int noAprilTags;
    private State state;
    private LinearInterpolator linterp;
    private boolean shouldCancel;
    private boolean hasCompleted;

    public ShootVisionTask() {
        this(false);
    }

    @Override
    public void begin() {        
        super.begin();
        this.vision = this.getInjector().getInstance(OffboardVisionManager.class);     
        this.linterp = new LinearInterpolator(TuningConstants.SHOOTING_POINTS, TuningConstants.SHOOTING_ANGLES);
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, true);
        this.AppendTask(new VisionTurningTask(VisionTurningTask.TurnType.AprilTagCentering, DigitalOperation.VisionFindSpeakerAprilTagRear));

    }

    @Override
    protected void finishedTask(IControlTask finishedTask) {
        super.finishedTask(finishedTask);
        if (finishedTask instanceof VisionTurningTask) {
            double distance = vision.getAprilTagXOffset();
            if ( // if distance is out of range cancel, avoid inaccuracy in interpolation
                distance < TuningConstants.SHOOTING_POINTS[0] ||
                distance > TuningConstants.SHOOTING_POINTS[TuningConstants.SHOOTING_POINTS.length - 1])
            {
                this.state = State.Cancel;
                this.shouldCancel = true;
            }
            else {
                double wristAngle = this.linterp.sample(distance);
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, wristAngle);
                if (Math.abs(this.arm.getWristPosition() - wristAngle) <= TuningConstants.WRIST_ACCURACY_THRESHOLD) {
                    this.hasCompleted = true;
                }

            }
            this.AppendTask(new SetEndEffectorAngleTask(this.wristAngle));
        }
        if (finishedTask instanceof SequentialTask) {
            this.hasCompleted = true;
        }
    }

    @Override
    public void update() {
        super.update();
    }

    @Override
    public boolean shouldCancel() {
        if(shouldCancel) {
            return true;
        }

        return super.shouldCancel();    
    }

    @Override
    public boolean hasCompleted() {
        if (hasCompleted) {
            return true;
        }
        return super.hasCompleted();
    }

    @Override
    public void end() {
        super.end();
    }
}