package frc.robot;

/**
 * Keys describing logging 
 */
public enum LoggingKey
{
    RobotState("r.state", 1, true),
    RobotTime("r.time", 1, true),
    RobotMatch("r.match", 50),
    RobotCrash("r.crash", true),
    DriverMode("driver.mode", 1, true),
    DriverActiveMacros("driver.activeMacros", true),
    DriverActiveShifts("driver.activeShifts"),
    AutonomousSelection("auto.selected"),
    AutonomousDSMessage("auto.dsMessage"),
    OffboardVisionRRTargetDistance("vision.rr_distance"),
    OffboardVisionRRTargetHorizontalAngle("vision.rr_horizontalAngle"),
    OffboardVisionAprilTagXOffset("vision.atXOffset"),
    OffboardVisionAprilTagYOffset("vision.atYOffset"),
    OffboardVisionAprilTagZOffset("vision.atZOffset"),
    OffboardVisionAprilTagYaw("vision.atYaw"),
    OffboardVisionAprilTagPitch("vision.atPitch"),
    OffboardVisionAprilTagRoll("vision.atRoll"),
    OffboardVisionAprilTagId("vision.atId"),
    OffboardVisionProcessingMode("vision.processingMode"),
    OffboardVisionEnableStream("vision.enableStream"),
    OffboardVisionMissedHeartbeats("vision.missedHeartbeats"),
    PowerCurrent("power.curr"),
    PowerCurrentFloatingAverage("power.currFltAvg"),
    PowerBatteryVoltage("power.battV"),
    NavxStartingAngle("navx.startingAngle"),
    PigeonState("pigeon.state"),
    PigeonYaw("pigeon.yaw"),
    PigeonPitch("pigeon.pitch"),
    PigeonPitchOffset("pigeon.pitchOffset"),
    PigeonRollOffset("pigeon.rollOffset"),
    PigeonYawOffset("pigeon.yawOffset"),
    PigeonRoll("pigeon.roll"),
    PigeonStartingYaw("pigeon.startingYaw"),
    PigeonYawRate("pigeon.yawRate"),
    PigeonPitchRate("pigeon.pitchRate"),
    PigeonRollRate("pigeon.rollRate"),
    NavxConnected("navx.isConnected"),
    NavxAngle("navx.angle"),
    NavxPitch("navx.pitch"),
    NavxRoll("navx.roll"),
    NavxYaw("navx.yaw"),
    NavxX("navx.x"),
    NavxY("navx.y"),
    NavxZ("navx.z"),

    IntakeMotorVelocity("ee.int_vel"),
    IntakeMotorPercentOutput("ee.int_per_out)"),
    ShooterFlywheelPosition("ee.fw_pos"),
    ShooterFlywheelVelocity("ee.fw_vel"),
    ShooterFlywheelError("ee.fw_err"),
    IntakeThroughBeamSensorValue("ee.thr_bem_vol"),
    IntakeThroughBeamBroken("ee.thr_bem_brk"),
    FlywheelPower("ee_fw_pow"),
    FlywheelDesiredVelocity("ee.fw_vel_setpt"),



    DriveTrainDesiredAngle("dt.angle_goal", 1),
    DriveTrainAngle("dt.angle", 1),
    DriveTrainXPosition("dt.xpos", 1, true),
    DriveTrainYPosition("dt.ypos", 1, true),
    DriveTrainXPositionGoal("dt.xpos_goal", 1),
    DriveTrainYPositionGoal("dt.ypos_goal", 1),
    DriveTrainAngleGoal("dt.angle_pathgoal", 1),
    DriveTrainXVelocityGoal("dt.xvel_goal", 1),
    DriveTrainYVelocityGoal("dt.yvel_goal", 1),
    DriveTrainAngleVelocityGoal("dt.anglevel_goal", 1),
    DriveTrainFieldOriented("dt.field_oriented", 1),
    DriveTrainMaintainOrientation("dt.maintain_orientation", 1),

    DriveTrainAbsoluteEncoderAngle1("dt.absenc_ang1", 1),
    DriveTrainDriveVelocity1("dt.drive_vel1", 1),
    DriveTrainDrivePosition1("dt.drive_pos1", 1),
    DriveTrainDriveError1("dt.drive_err1", 1), // SDS-only
    DriveTrainDriveVelocityGoal1("dt.drive_goal1", 1),
    DriveTrainSteerVelocity1("dt.steer_vel1", 1),
    DriveTrainSteerPosition1("dt.steer_pos1", 1), // SDS-only
    DriveTrainSteerAngle1("dt.steer_ang1", 1),
    DriveTrainSteerError1("dt.steer_err1", 1), // SDS-only
    DriveTrainSteerPositionGoal1("dt.steer_goal1", 1),

    DriveTrainAbsoluteEncoderAngle2("dt.absenc_ang2", 1),
    DriveTrainDriveVelocity2("dt.drive_vel2", 1),
    DriveTrainDrivePosition2("dt.drive_pos2", 1),
    DriveTrainDriveError2("dt.drive_err2", 1), // SDS-only
    DriveTrainDriveVelocityGoal2("dt.drive_goal2", 1),
    DriveTrainSteerVelocity2("dt.steer_vel2", 1),
    DriveTrainSteerPosition2("dt.steer_pos2", 1), // SDS-only
    DriveTrainSteerAngle2("dt.steer_ang2", 1),
    DriveTrainSteerError2("dt.steer_err2", 1), // SDS-only
    DriveTrainSteerPositionGoal2("dt.steer_goal2", 1),

    DriveTrainAbsoluteEncoderAngle3("dt.absenc_ang3", 1),
    DriveTrainDriveVelocity3("dt.drive_vel3", 1),
    DriveTrainDrivePosition3("dt.drive_pos3", 1),
    DriveTrainDriveError3("dt.drive_err3", 1), // SDS-only
    DriveTrainDriveVelocityGoal3("dt.drive_goal3", 1),
    DriveTrainSteerVelocity3("dt.steer_vel3", 1),
    DriveTrainSteerPosition3("dt.steer_pos3", 1), // SDS-only
    DriveTrainSteerAngle3("dt.steer_ang3", 1),
    DriveTrainSteerError3("dt.steer_err3", 1), // SDS-only
    DriveTrainSteerPositionGoal3("dt.steer_goal3", 1),

    DriveTrainAbsoluteEncoderAngle4("dt.absenc_ang4", 1),
    DriveTrainDriveVelocity4("dt.drive_vel4", 1),
    DriveTrainDrivePosition4("dt.drive_pos4", 1),
    DriveTrainDriveError4("dt.drive_err4", 1), // SDS-only
    DriveTrainDriveVelocityGoal4("dt.drive_goal4", 1),
    DriveTrainSteerVelocity4("dt.steer_vel4", 1),
    DriveTrainSteerPosition4("dt.steer_pos4", 1), // SDS-only
    DriveTrainSteerAngle4("dt.steer_ang4", 1),
    DriveTrainSteerError4("dt.steer_err4", 1), // SDS-only
    DriveTrainSteerPositionGoal4("dt.steer_goal4", 1),

    CompressorPreassure("com.pres");

    //Arm Stuff
        //chm = Chain Motor
    LeftMotorPosition("chm.position");
    LeftMotorVelocity("chm.velocity");

    LeftLAPosition("la.position");
    LeftLAVelocity("la.velocity");

    public final String value;
    public final int loggingFrequency;
    public final boolean shouldLogToCsv;
    private LoggingKey(String value)
    {
        this(value, TuningConstants.DEFAULT_LOGGING_FREQUENCY, false);
    }

    private LoggingKey(String value, int loggingFrequency)
    {
        this(value, loggingFrequency, false);
    }

    private LoggingKey(String value, boolean shouldLogToCsv)
    {
        this(value, TuningConstants.DEFAULT_LOGGING_FREQUENCY, shouldLogToCsv);
    }

    private LoggingKey(String value, int loggingFrequency, boolean shouldLogToCsv)
    {
        if (loggingFrequency <= 0)
        {
            loggingFrequency = TuningConstants.DEFAULT_LOGGING_FREQUENCY;
        }

        this.value = value;
        this.loggingFrequency = loggingFrequency;
        this.shouldLogToCsv = shouldLogToCsv;
    }
}
