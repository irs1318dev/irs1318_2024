package frc.robot.driver.controltasks;

import frc.lib.helpers.LinearInterpolator;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ArmMechanism;
import frc.robot.mechanisms.OffboardVisionManager;

public class ShootVisionTask extends DecisionSequentialTask{

    public enum State {
        ReadAprilTag,
        SetWrist,
        Cancel
    }

    private ArmMechanism arm;
    private OffboardVisionManager vision;
    private int noAprilTags;
    private State state;
    private LinearInterpolator linterp;

    @Override
    public void begin() {
        
        super.begin();

        this.vision = this.getInjector().getInstance(OffboardVisionManager.class);     
        this.linterp = new LinearInterpolator(TuningConstants.SHOOTING_POINTS, TuningConstants.SHOOTING_ANGLES);
        this.arm = this.getInjector().getInstance(ArmMechanism.class);

        this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, true);

    }

    @Override
    public void update() {
        if (this.state == State.ReadAprilTag) {
            if (vision.getAprilTagId() == null) {
                this.noAprilTags++;
            }
            if (this.noAprilTags > TuningConstants.NOT_FOUND_APRILTAG_THRESHOLD) {
                this.state = State.Cancel;
            }
            else {
                this.state = State.SetWrist;
            }
        }
        else if (this.state == State.SetWrist) {
            double distance = vision.getAprilTagXOffset();
            if ( // if distance is out of range cancel, avoid inaccuracy in interpolation
                distance < TuningConstants.SHOOTING_POINTS[0] ||
                distance > TuningConstants.SHOOTING_POINTS[TuningConstants.SHOOTING_POINTS.length - 1])
            {
                this.state = State.Cancel;
            }
            else {
                double wristAngle = this.linterp.sample(distance);
                this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, wristAngle);
            }
        }
        if (this.state == State.Cancel) {
            shouldCancel();
        }
        super.update();
    }

    @Override
    public boolean shouldCancel() {
        return super.shouldCancel();
    }
}
