package frc.robot;

/**
 * Keys describing logging 
 */
public enum LoggingKey
{
    RobotState("r.state", false, 1, true),
    RobotTime("r.time", false, 1, true),
    RobotMatch("r.match", false, 50),
    RobotCrash("r.crash", false, true),
    DriverMode("driver.mode", false, 1, true),
    DriverActiveMacros("driver.activeMacros", false, 1, true),
    DriverActiveShifts("driver.activeShifts", false),
    AutonomousSelection("auto.selected", false),
    AutonomousDSMessage("auto.dsMessage", false),
    OffboardVisionAprilTagXOffset("vision.atXOffset", true, 1),
    OffboardVisionAprilTagYOffset("vision.atYOffset", true, 1),
    OffboardVisionAprilTagZOffset("vision.atZOffset", true, 1),
    OffboardVisionAprilTagYaw("vision.atYaw", true, 1),
    OffboardVisionAprilTagPitch("vision.atPitch", true, 1),
    OffboardVisionAprilTagRoll("vision.atRoll", true, 1),
    OffboardVisionAprilTagId("vision.atId", true, 1),
    OffboardVisionProcessingMode("vision.processingMode", false, 1),
    OffboardVisionEnableStream("vision.enableStream", false, 1),
    OffboardVisionDesiredTarget("vision.desiredTarget", false, 1),
    OffboardVisionMissedHeartbeats("vision.missedHeartbeats", true, 1),
    PowerCurrent("power.curr", true),
    PowerCurrentFloatingAverage("power.currFltAvg", false),
    PowerBatteryVoltage("power.battV", true),
    PowerBatteryVoltageFiltered("power.battVFilt", false),
    PigeonYaw("pigeon.yaw", true),
    PigeonPitch("pigeon.pitch", true),
    PigeonPitchOffset("pigeon.pitchOffset", false),
    PigeonRollOffset("pigeon.rollOffset", false),
    PigeonYawOffset("pigeon.yawOffset", false),
    PigeonRoll("pigeon.roll", true),
    PigeonStartingYaw("pigeon.startingYaw", false),
    PigeonYawRate("pigeon.yawRate", true),
    PigeonPitchRate("pigeon.pitchRate", true),
    PigeonRollRate("pigeon.rollRate", true),

    DriveTrainDesiredAngle("dt.angle_goal", false),
    DriveTrainAngle("dt.angle", false),
    DriveTrainXPosition("dt.xpos", false, 1, true),
    DriveTrainYPosition("dt.ypos", false, 1, true),
    DriveTrainXPositionGoal("dt.xpos_goal", false, true),
    DriveTrainYPositionGoal("dt.ypos_goal", false, true),
    DriveTrainAngleGoal("dt.angle_pathgoal", false),
    DriveTrainXVelocityGoal("dt.xvel_goal", false),
    DriveTrainYVelocityGoal("dt.yvel_goal", false),
    DriveTrainAngleVelocityGoal("dt.anglevel_goal", false),
    DriveTrainFieldOriented("dt.field_oriented", false),
    DriveTrainMaintainOrientation("dt.maintain_orientation", false),

    DriveTrainAbsoluteEncoderAngle1("dt.absenc_ang1", true),
    DriveTrainDriveVelocity1("dt.drive_vel1", true),
    DriveTrainDrivePosition1("dt.drive_pos1", true),
    DriveTrainDriveError1("dt.drive_err1", true), // SDS-only
    DriveTrainDriveVelocityGoal1("dt.drive_goal1", false),
    DriveTrainSteerVelocity1("dt.steer_vel1", true),
    DriveTrainSteerPosition1("dt.steer_pos1", true), // SDS-only
    DriveTrainSteerAngle1("dt.steer_ang1", false),
    DriveTrainSteerError1("dt.steer_err1", true), // SDS-only
    DriveTrainSteerPositionGoal1("dt.steer_goal1", false),

    DriveTrainAbsoluteEncoderAngle2("dt.absenc_ang2", true),
    DriveTrainDriveVelocity2("dt.drive_vel2", true),
    DriveTrainDrivePosition2("dt.drive_pos2", true),
    DriveTrainDriveError2("dt.drive_err2", true), // SDS-only
    DriveTrainDriveVelocityGoal2("dt.drive_goal2", false),
    DriveTrainSteerVelocity2("dt.steer_vel2", true),
    DriveTrainSteerPosition2("dt.steer_pos2", true), // SDS-only
    DriveTrainSteerAngle2("dt.steer_ang2", false),
    DriveTrainSteerError2("dt.steer_err2", true), // SDS-only
    DriveTrainSteerPositionGoal2("dt.steer_goal2", false),

    DriveTrainAbsoluteEncoderAngle3("dt.absenc_ang3", true),
    DriveTrainDriveVelocity3("dt.drive_vel3", true),
    DriveTrainDrivePosition3("dt.drive_pos3", true),
    DriveTrainDriveError3("dt.drive_err3", true), // SDS-only
    DriveTrainDriveVelocityGoal3("dt.drive_goal3", false),
    DriveTrainSteerVelocity3("dt.steer_vel3", true),
    DriveTrainSteerPosition3("dt.steer_pos3", true), // SDS-only
    DriveTrainSteerAngle3("dt.steer_ang3", false),
    DriveTrainSteerError3("dt.steer_err3", true), // SDS-only
    DriveTrainSteerPositionGoal3("dt.steer_goal3", false),
    
    DriveTrainAbsoluteEncoderAngle4("dt.absenc_ang4", true),
    DriveTrainDriveVelocity4("dt.drive_vel4", true),
    DriveTrainDrivePosition4("dt.drive_pos4", true),
    DriveTrainDriveError4("dt.drive_err4", true), // SDS-only
    DriveTrainDriveVelocityGoal4("dt.drive_goal4", false),
    DriveTrainSteerVelocity4("dt.steer_vel4", true),
    DriveTrainSteerPosition4("dt.steer_pos4", true), // SDS-only
    DriveTrainSteerAngle4("dt.steer_ang4", false),
    DriveTrainSteerError4("dt.steer_err4", true), // SDS-only
    DriveTrainSteerPositionGoal4("dt.steer_goal4", false),

    // End Effector stuff
    IntakeMotorVelocity("ee.int_vel", true),
    IntakeMotorPercentOutput("ee.int_per_out)", false),
    ShooterNearFlywheelPosition("ee.nr_fly_pos", true),
    ShooterNearFlywheelVelocity("ee.nr_fly_vel", true),
    ShooterNearFlywheelError("ee.nr_fly_err", false),
    ShooterFarFlywheelPosition("ee.fr_fly_pos", true),
    ShooterFarFlywheelVelocity("ee.fr_fly_vel", true),
    ShooterFarFlywheelError("ee.fr_fly_err", false),
    IntakeThroughBeamSensorValue("ee.thr_bem_vol", true),
    IntakeThroughBeamBroken("ee.thr_bem_brk", false),
    ShooterFlywheelPower("ee_fw_pow", false),
    ShooterNearFlywheelDesiredVelocity("ee.nr_fly_vel_setpt", false),
    ShooterFarFlywheelDesiredVelocity("ee.fr_fly_vel_setpt", false),
    ShooterSpunUp("ee.shooter_spunup", false),

    // Arm stuff
    ArmClamped("arm.clamped", false, true),
    ArmProtectionState("arm.protect", false, false),
    ArmShoulderSetpointTMP("arm.shPosDesCurr", false, true),
    ArmShoulderSetpoint("arm.shPosDesired", false, true),
    ArmShoulderPosition("arm.shPos", true, true),
    ArmShoulderVelocity("arm.shVel", true, true),
    ArmShoulderError("arm.shErr", false, true),
    ArmShoulderOutput("arm.shOutput", false, true),
    ArmShoulderPowerAverage("arm.shPowAvg", false, true),
    ArmShoulderMotorPowerDiscrepancy("arm.shPowDiscrp", false, true),
    ArmShoulderVelocityAverage("arm.shVelAvg", false, true),
    ArmShoulderStalled("arm.shStalled", false, true),
    ArmWristSetpointTMP("arm.wrPosDesCurr", false, true),
    ArmWristSetpoint("arm.wrPosDesired", false, true),
    ArmWristPosition("arm.wrPos", true, true),
    ArmWristVelocity("arm.wrVel", true, true),
    ArmWristError("arm.wrErr", false, true),
    ArmWristOutput("arm.wrOutput", false, true),
    ArmWristVelocityAverage("arm.wrVelAvg", false, true),
    ArmWristPowerAverage("arm.wrPowAvg", false, true),
    ArmWristStalled("arm.wrStalled", false, true),
    ArmWristLimitSwitch("arm.wrLimitSwitch", true, true),

    ArmShoulderPosAdjustment("arm.sh_pos_adj", false),
    ArmWristPosAdjustment("arm.wr_pos_adj", false),
    ArmWristAbsoluteEncoderPosition("arm.wrAbsPos", true, true),
    ArmShoulderAbsoluteEncoderPosition("arm.shAbsPos", true, true),

    ArmExtensionBreaking("arm.exten_broken", false),
    ArmFixedWithIK("arm.fixed_with_IK", false),
    ArmIntakeTopAbsX("arm.intake_top_abs_x", false),
    ArmIntakeTopAbsZ("arm.intake_top_abs_z", false),
    ArmHittingRobot("arm.hit_robot", false),
    ArmIntakeBottomAbsX("arm.intake_bot_abs_x", false),
    ArmIntakeBottomAbsZ("arm.intake_bot_abs_z", false),
    ArmShooterBottomAbsX("arm.shooter_bot_abs_x", false),
    ArmShooterBottomAbsZ("arm.shooter_bot_abs_z", false),
    ArmShooterTopAbsX("arm.shooter_top_abs_x", false),
    ArmShooterTopAbsZ("arm.shooter_top_abs_z", false),
    ArmWristAbsX("arm.wrist_abs_x", false),
    ArmWristAbsZ("arm.wrist_abs_z", false),

    ArmExtensionType("arm.extension_type", false),
    ArmWristIKDesired("arm.wrist_ik_desired", false),
    ArmShoulderIKDesired("arm.sh_ik_desired", false),
    ArmShoulderLastLegal("arm.sh_lst_legal_pos", false),

    ActWristDesired("arm.sh_nt_tsk_wr_ang", false),
    ArmTheta4("arm.theta_4", false),

    // Climber stuff
    ClimberMotorPower("cl.motor", false),
    ClimberServoPosition("cl.servo", false),
    ClimberLimitSwitch("cl.limit", true);

    public final String value;
    public final boolean isInput;
    public final int loggingFrequency;
    public final boolean shouldLogToCsv;
    private LoggingKey(String value)
    {
        this(value, false, TuningConstants.DEFAULT_LOGGING_FREQUENCY, false);
    }

    private LoggingKey(String value, boolean isInput)
    {
        this(value, isInput, TuningConstants.DEFAULT_LOGGING_FREQUENCY, false);
    }

    private LoggingKey(String value, boolean isInput, int loggingFrequency)
    {
        this(value, isInput, loggingFrequency, false);
    }

    private LoggingKey(String value, boolean isInput, boolean shouldLogToCsv)
    {
        this(value, isInput, TuningConstants.DEFAULT_LOGGING_FREQUENCY, shouldLogToCsv);
    }

    private LoggingKey(String value, boolean isInput, int loggingFrequency, boolean shouldLogToCsv)
    {
        if (loggingFrequency <= 0)
        {
            loggingFrequency = TuningConstants.DEFAULT_LOGGING_FREQUENCY;
        }

        this.value = value;
        this.isInput = isInput;
        this.loggingFrequency = loggingFrequency;
        this.shouldLogToCsv = shouldLogToCsv;
    }
}
