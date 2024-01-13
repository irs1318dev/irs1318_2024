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



    DriveTrainDesiredAngle("dt.angle_goal"),
    DriveTrainAngle("dt.angle"),
    DriveTrainXPosition("dt.xpos", true),
    DriveTrainYPosition("dt.ypos", true),
    DriveTrainXPositionGoal("dt.xpos_goal"),
    DriveTrainYPositionGoal("dt.ypos_goal"),
    DriveTrainAngleGoal("dt.angle_pathgoal"),
    DriveTrainXVelocityGoal("dt.xvel_goal"),
    DriveTrainYVelocityGoal("dt.yvel_goal"),
    DriveTrainAngleVelocityGoal("dt.anglevel_goal"),
    DriveTrainFieldOriented("dt.field_oriented"),
    DriveTrainMaintainOrientation("dt.maintain_orientation"),

    DriveTrainAbsoluteEncoderAngle1("dt.absenc_ang1"),
    DriveTrainDriveVelocity1("dt.drive_vel1"),
    DriveTrainDrivePosition1("dt.drive_pos1"),
    DriveTrainDriveError1("dt.drive_err1"), // SDS-only
    DriveTrainDriveVelocityGoal1("dt.drive_goal1"),
    DriveTrainSteerVelocity1("dt.steer_vel1"),
    DriveTrainSteerPosition1("dt.steer_pos1"), // SDS-only
    DriveTrainSteerAngle1("dt.steer_ang1"),
    DriveTrainSteerError1("dt.steer_err1"), // SDS-only
    DriveTrainSteerPositionGoal1("dt.steer_goal1"),

    DriveTrainAbsoluteEncoderAngle2("dt.absenc_ang2"),
    DriveTrainDriveVelocity2("dt.drive_vel2"),
    DriveTrainDrivePosition2("dt.drive_pos2"),
    DriveTrainDriveError2("dt.drive_err2"), // SDS-only
    DriveTrainDriveVelocityGoal2("dt.drive_goal2"),
    DriveTrainSteerVelocity2("dt.steer_vel2"),
    DriveTrainSteerPosition2("dt.steer_pos2"), // SDS-only
    DriveTrainSteerAngle2("dt.steer_ang2"),
    DriveTrainSteerError2("dt.steer_err2"), // SDS-only
    DriveTrainSteerPositionGoal2("dt.steer_goal2"),

    DriveTrainAbsoluteEncoderAngle3("dt.absenc_ang3"),
    DriveTrainDriveVelocity3("dt.drive_vel3"),
    DriveTrainDrivePosition3("dt.drive_pos3"),
    DriveTrainDriveError3("dt.drive_err3"), // SDS-only
    DriveTrainDriveVelocityGoal3("dt.drive_goal3"),
    DriveTrainSteerVelocity3("dt.steer_vel3"),
    DriveTrainSteerPosition3("dt.steer_pos3"), // SDS-only
    DriveTrainSteerAngle3("dt.steer_ang3"),
    DriveTrainSteerError3("dt.steer_err3"), // SDS-only
    DriveTrainSteerPositionGoal3("dt.steer_goal3"),

    DriveTrainAbsoluteEncoderAngle4("dt.absenc_ang4"),
    DriveTrainDriveVelocity4("dt.drive_vel4"),
    DriveTrainDrivePosition4("dt.drive_pos4"),
    DriveTrainDriveError4("dt.drive_err4"), // SDS-only
    DriveTrainDriveVelocityGoal4("dt.drive_goal4"),
    DriveTrainSteerVelocity4("dt.steer_vel4"),
    DriveTrainSteerPosition4("dt.steer_pos4"), // SDS-only
    DriveTrainSteerAngle4("dt.steer_ang4"),
    DriveTrainSteerError4("dt.steer_err4"), // SDS-only
    DriveTrainSteerPositionGoal4("dt.steer_goal4"),

    CompressorPreassure("com.pres");

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
