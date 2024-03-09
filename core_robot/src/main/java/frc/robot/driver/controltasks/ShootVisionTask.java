// package frc.robot.driver.controltasks;

// import frc.lib.helpers.LinearInterpolator;
// import frc.robot.TuningConstants;
// import frc.robot.driver.AnalogOperation;
// import frc.robot.driver.DigitalOperation;
// import frc.robot.mechanisms.ArmMechanism;
// import frc.robot.mechanisms.OffboardVisionManager;

// public class ShootVisionTask extends IControlTask {

//     public enum State {
//         ReadAprilTag,
//         SetWrist,
//         Cancel
//     }

//     private OffboardVisionManager vision;
//     private int noAprilTags;
//     private State state;
//     private LinearInterpolator linterp;
//     private ArmMechanism arm;
//     private boolean shouldCancel;
//     private boolean hasCompleted;

//     @Override
//     public void begin() {
        
//         super.begin();

//         this.vision = this.getInjector().getInstance(OffboardVisionManager.class);     
//         this.linterp = new LinearInterpolator(TuningConstants.SHOOTING_POINTS, TuningConstants.SHOOTING_ANGLES);
//         this.arm = this.getInjector().getInstance(ArmMechanism.class);
//         this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, true);
//     }

//     @Override
//     public void update() {

//         if (this.state == State.ReadAprilTag) {
//             this.setDigitalOperationState(DigitalOperation.VisionFindSpeakerAprilTagRear, false);
//             if (vision.getAprilTagId() == null) {
//                 this.noAprilTags++;
//             }
//             if (this.noAprilTags > TuningConstants.NOT_FOUND_APRILTAG_THRESHOLD) {
//                 this.state = State.Cancel;
//                 shouldCancel = true;
//             }
//             else {
//                 this.state = State.SetWrist;
//             }
//         }
//         else if (this.state == State.SetWrist) {
//             double distance = vision.getAprilTagXOffset();
//             if ( // if distance is out of range cancel, avoid inaccuracy in interpolation
//                 distance < TuningConstants.SHOOTING_POINTS[0] ||
//                 distance > TuningConstants.SHOOTING_POINTS[TuningConstants.SHOOTING_POINTS.length - 1])
//             {
//                 this.state = State.Cancel;
//                 shouldCancel = true;
//             }
//             else {
//                 double wristAngle = this.linterp.sample(distance);
//                 this.setAnalogOperationState(AnalogOperation.ArmWristPositionSetpoint, wristAngle);
//                 if (Math.abs(this.arm.getWristPosition() - wristAngle) <= TuningConstants.WRIST_ACCURACY_THRESHOLD) {
//                     hasCompleted = true;
//                 }

//             }
//         }

//         super.update();
//     }

//     @Override
//     public boolean shouldCancel() {
//         if(shouldCancel) {
//             return true;
//         }

//         return super.shouldCancel();    
//     }

//     @Override
//     public boolean hasCompleted() {
//         if (hasCompleted) {
//             return true;
//         }
//         return super.hasCompleted();
//     }

//     @Override
//     public void end() {
//         super.end();
//     }
// }