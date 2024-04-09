package frc.robot;

import frc.lib.robotprovider.LoggingType;

/**
 * Keys describing logging 
 */
public enum LoggingKey
{
    RobotState("r/state", LoggingType.String, false, 1, true),
    RobotTime("r/time", LoggingType.Number, false, 1, true),
    RobotMatch("r/match", LoggingType.String, false, 50),
    RobotCrash("r/crash", LoggingType.String, false, true),
    DriverMode("driver/mode", LoggingType.String, false, 1, true),
    DriverActiveMacros("driver/activeMacros", LoggingType.String, false, 1, true),
    DriverActiveShifts("driver/activeShifts", LoggingType.String, false),
    AutonomousSelection("auto/selected", LoggingType.String, false),
    AutonomousDSMessage("auto/dsMessage", LoggingType.String, false),
    OffboardVisionAprilTagXOffset("vision/atXOffset", LoggingType.String, true, 1),
    OffboardVisionAprilTagYOffset("vision/atYOffset", LoggingType.String, true, 1),
    OffboardVisionAprilTagZOffset("vision/atZOffset", LoggingType.String, true, 1),
    OffboardVisionAprilTagYaw("vision/atYaw", LoggingType.String, true, 1),
    OffboardVisionAprilTagPitch("vision/atPitch", LoggingType.String, true, 1),
    OffboardVisionAprilTagRoll("vision/atRoll", LoggingType.String, true, 1),
    OffboardVisionAprilTagId("vision/atId", LoggingType.String, true, 1),
    OffboardVisionProcessingMode("vision/processingMode", LoggingType.Integer, false, 1),
    OffboardVisionEnableStream("vision/enableStream", LoggingType.Boolean, false, 1),
    OffboardVisionDesiredTarget("vision/desiredTarget", LoggingType.String, false, 1),
    OffboardVisionMissedHeartbeats("vision/missedHeartbeats", LoggingType.Number, true, 1),
    OffboardVisionExcessiveMissedHeartbeats("vision/missedTooManyHeartbeats", LoggingType.Boolean, false, 1),
    PowerCurrent("power/curr", LoggingType.Number, true),
    PowerCurrentFloatingAverage("power/currFltAvg", LoggingType.Number, false),
    PowerBatteryVoltage("power/battV", LoggingType.Number, true),
    // PowerBatteryVoltageFiltered("power/battVFilt", LoggingType.Number, false),
    PigeonYaw("pigeon/yaw", LoggingType.Number, true),
    PigeonPitch("pigeon/pitch", LoggingType.Number, true),
    PigeonPitchOffset("pigeon/pitchOffset", LoggingType.Number, false),
    PigeonRollOffset("pigeon/rollOffset", LoggingType.Number, false),
    PigeonYawOffset("pigeon/yawOffset", LoggingType.Number, false),
    PigeonRoll("pigeon/roll", LoggingType.Number, true),
    PigeonStartingYaw("pigeon/startingYaw", LoggingType.Number, false),
    PigeonYawRate("pigeon/yawRate", LoggingType.Number, true),
    PigeonPitchRate("pigeon/pitchRate", LoggingType.Number, true),
    PigeonRollRate("pigeon/rollRate", LoggingType.Number, true),

    DriveTrainDesiredAngle("dt/angle_goal", LoggingType.Number, false),
    DriveTrainAngle("dt/angle", LoggingType.Number, false),
    DriveTrainXPosition("dt/xpos", LoggingType.Number, false, 1, true),
    DriveTrainYPosition("dt/ypos", LoggingType.Number, false, 1, true),
    DriveTrainXPositionGoal("dt/xpos_goal", LoggingType.Number, false, true),
    DriveTrainYPositionGoal("dt/ypos_goal", LoggingType.Number, false, true),
    DriveTrainAngleGoal("dt/angle_pathgoal", LoggingType.Number, false),
    DriveTrainXVelocityGoal("dt/xvel_goal", LoggingType.Number, false),
    DriveTrainYVelocityGoal("dt/yvel_goal", LoggingType.Number, false),
    DriveTrainAngleVelocityGoal("dt/anglevel_goal", LoggingType.Number, false),
    DriveTrainFieldOriented("dt/field_oriented", LoggingType.Boolean, false),
    DriveTrainMaintainOrientation("dt/maintain_orientation", LoggingType.Boolean, false),

    DriveTrainAbsoluteEncoderAngle1("dt/absenc_ang1", LoggingType.Number, true),
    DriveTrainDriveVelocity1("dt/drive_vel1", LoggingType.Number, true),
    DriveTrainDrivePosition1("dt/drive_pos1", LoggingType.Number, true),
    DriveTrainDriveError1("dt/drive_err1", LoggingType.Number, true), // SDS-only
    DriveTrainDriveVelocityGoal1("dt/drive_goal1", LoggingType.Number, false),
    DriveTrainSteerVelocity1("dt/steer_vel1", LoggingType.Number, true),
    DriveTrainSteerPosition1("dt/steer_pos1", LoggingType.Number, true), // SDS-only
    DriveTrainSteerAngle1("dt/steer_ang1", LoggingType.Number, false),
    DriveTrainSteerError1("dt/steer_err1", LoggingType.Number, true), // SDS-only
    DriveTrainSteerPositionGoal1("dt/steer_goal1", LoggingType.String, false),

    DriveTrainAbsoluteEncoderAngle2("dt/absenc_ang2", LoggingType.Number, true),
    DriveTrainDriveVelocity2("dt/drive_vel2", LoggingType.Number, true),
    DriveTrainDrivePosition2("dt/drive_pos2", LoggingType.Number, true),
    DriveTrainDriveError2("dt/drive_err2", LoggingType.Number, true), // SDS-only
    DriveTrainDriveVelocityGoal2("dt/drive_goal2", LoggingType.Number, false),
    DriveTrainSteerVelocity2("dt/steer_vel2", LoggingType.Number, true),
    DriveTrainSteerPosition2("dt/steer_pos2", LoggingType.Number, true), // SDS-only
    DriveTrainSteerAngle2("dt/steer_ang2", LoggingType.Number, false),
    DriveTrainSteerError2("dt/steer_err2", LoggingType.Number, true), // SDS-only
    DriveTrainSteerPositionGoal2("dt/steer_goal2", LoggingType.String, false),

    DriveTrainAbsoluteEncoderAngle3("dt/absenc_ang3", LoggingType.Number, true),
    DriveTrainDriveVelocity3("dt/drive_vel3", LoggingType.Number, true),
    DriveTrainDrivePosition3("dt/drive_pos3", LoggingType.Number, true),
    DriveTrainDriveError3("dt/drive_err3", LoggingType.Number, true), // SDS-only
    DriveTrainDriveVelocityGoal3("dt/drive_goal3", LoggingType.Number, false),
    DriveTrainSteerVelocity3("dt/steer_vel3", LoggingType.Number, true),
    DriveTrainSteerPosition3("dt/steer_pos3", LoggingType.Number, true), // SDS-only
    DriveTrainSteerAngle3("dt/steer_ang3", LoggingType.Number, false),
    DriveTrainSteerError3("dt/steer_err3", LoggingType.Number, true), // SDS-only
    DriveTrainSteerPositionGoal3("dt/steer_goal3", LoggingType.String, false),
    
    DriveTrainAbsoluteEncoderAngle4("dt/absenc_ang4", LoggingType.Number, true),
    DriveTrainDriveVelocity4("dt/drive_vel4", LoggingType.Number, true),
    DriveTrainDrivePosition4("dt/drive_pos4", LoggingType.Number, true),
    DriveTrainDriveError4("dt/drive_err4", LoggingType.Number, true), // SDS-only
    DriveTrainDriveVelocityGoal4("dt/drive_goal4", LoggingType.Number, false),
    DriveTrainSteerVelocity4("dt/steer_vel4", LoggingType.Number, true),
    DriveTrainSteerPosition4("dt/steer_pos4", LoggingType.Number, true), // SDS-only
    DriveTrainSteerAngle4("dt/steer_ang4", LoggingType.Number, false),
    DriveTrainSteerError4("dt/steer_err4", LoggingType.Number, true), // SDS-only
    DriveTrainSteerPositionGoal4("dt/steer_goal4", LoggingType.String, false),

    // End Effector stuff
    IntakeMotorVelocity("ee/int_vel", LoggingType.Number, true),
    IntakeMotorPercentOutput("ee/int_per_out", LoggingType.Number, false),
    ShooterNearFlywheelPosition("ee/nr_fly_pos", LoggingType.Number, true),
    ShooterNearFlywheelVelocity("ee/nr_fly_vel", LoggingType.Number, true),
    ShooterNearFlywheelError("ee/nr_fly_err", LoggingType.Number, false),
    ShooterFarFlywheelPosition("ee/fr_fly_pos", LoggingType.Number, true),
    ShooterFarFlywheelVelocity("ee/fr_fly_vel", LoggingType.Number, true),
    ShooterFarFlywheelError("ee/fr_fly_err", LoggingType.Number, false),
    IntakeThroughBeamSensorValue("ee/thr_bem_vol", LoggingType.Number, true),
    IntakeThroughBeamBroken("ee/thr_bem_brk", LoggingType.Boolean, false),
    ShooterFlywheelPower("ee/fw_pow", LoggingType.Number, false),
    ShooterNearFlywheelDesiredVelocity("ee/nr_fly_vel_setpt", LoggingType.Number, false),
    ShooterFarFlywheelDesiredVelocity("ee/fr_fly_vel_setpt", LoggingType.Number, false),
    ShooterSpunUp("ee/shooter_spunup", LoggingType.Boolean, false),

    // Arm stuff
    ArmClamped("arm/clamped", LoggingType.Boolean, false, true),
    ArmProtectionState("arm/protect", LoggingType.String, false, false),
    ArmShoulderSetpointTMP("arm/shPosDesCurr", LoggingType.Number, false, true),
    ArmShoulderSetpoint("arm/shPosDesired", LoggingType.Number, false, true),
    ArmShoulderPosition("arm/shPos", LoggingType.Number, true, true),
    ArmShoulderVelocity("arm/shVel", LoggingType.Number, true, true),
    ArmShoulderError("arm/shErr", LoggingType.Number, false, true),
    ArmShoulderOutput("arm/shOutput", LoggingType.Number, false, true),
    ArmShoulderPowerAverage("arm/shPowAvg", LoggingType.Number, false, true),
    ArmShoulderMotorPowerDiscrepancy("arm/shPowDiscrp", LoggingType.Boolean, false, true),
    ArmShoulderVelocityAverage("arm/shVelAvg", LoggingType.Number, false, true),
    ArmShoulderStalled("arm/shStalled", LoggingType.Boolean, false, true),
    ArmWristSetpointTMP("arm/wrPosDesCurr", LoggingType.Number, false, true),
    ArmWristSetpoint("arm/wrPosDesired", LoggingType.Number, false, true),
    ArmWristPosition("arm/wrPos", LoggingType.Number, true, true),
    ArmWristVelocity("arm/wrVel", LoggingType.Number, true, true),
    ArmWristError("arm/wrErr", LoggingType.Number, false, true),
    ArmWristOutput("arm/wrOutput", LoggingType.Number, false, true),
    ArmWristVelocityAverage("arm/wrVelAvg", LoggingType.Number, false, true),
    ArmWristPowerAverage("arm/wrPowAvg", LoggingType.Number, false, true),
    ArmWristStalled("arm/wrStalled", LoggingType.Boolean, false, true),
    ArmWristLimitSwitch("arm/wrLimitSwitch", LoggingType.Boolean, true, true),

    ArmShoulderPosAdjustment("arm/sh_pos_adj", LoggingType.Number, false),
    ArmWristPosAdjustment("arm/wr_pos_adj", LoggingType.Number, false),
    ArmWristAbsoluteEncoderPosition("arm/wrAbsPos", LoggingType.String, true, true),
    ArmShoulderAbsoluteEncoderPosition("arm/shAbsPos", LoggingType.String, true, true),

    ArmExtensionBreaking("arm/exten_broken", LoggingType.Boolean, false),
    ArmFixedWithIK("arm/fixed_with_IK", LoggingType.Boolean, false),
    ArmIntakeTopAbsX("arm/intake_top_abs_x", LoggingType.Number, false),
    ArmIntakeTopAbsZ("arm/intake_top_abs_z", LoggingType.Number, false),
    ArmHittingRobot("arm/hit_robot", LoggingType.Boolean, false),
    ArmIntakeBottomAbsX("arm/intake_bot_abs_x", LoggingType.Number, false),
    ArmIntakeBottomAbsZ("arm/intake_bot_abs_z", LoggingType.Number, false),
    ArmShooterBottomAbsX("arm/shooter_bot_abs_x", LoggingType.Number, false),
    ArmShooterBottomAbsZ("arm/shooter_bot_abs_z", LoggingType.Number, false),
    ArmShooterTopAbsX("arm/shooter_top_abs_x", LoggingType.Number, false),
    ArmShooterTopAbsZ("arm/shooter_top_abs_z", LoggingType.Number, false),
    ArmWristAbsX("arm/wrist_abs_x", LoggingType.Number, false),
    ArmWristAbsZ("arm/wrist_abs_z", LoggingType.Number, false),

    ArmExtensionType("arm/extension_type", LoggingType.String, false),
    ArmWristIKDesired("arm/wrist_ik_desired", LoggingType.Number, false),
    ArmShoulderIKDesired("arm/sh_ik_desired", LoggingType.Number, false),
    // ArmShoulderLastLegal("arm/sh_lst_legal_pos", LoggingType.Number, false),

    ActWristDesired("arm/sh_nt_tsk_wr_ang", LoggingType.Number, false),
    ArmTheta4("arm/theta_4", LoggingType.Number, false),

    // Climber stuff
    ClimberMotorPower("cl/motor", LoggingType.Number, false),
    ClimberServoPosition("cl/servo", LoggingType.Number, false),
    ClimberLimitSwitch("cl/limit", LoggingType.Boolean, true);

    public final String value;
    public final LoggingType type;
    public final boolean isInput;
    public final int loggingFrequency;
    public final boolean shouldLogToCsv;
    private LoggingKey(String value, LoggingType type)
    {
        this(value, type, false, TuningConstants.DEFAULT_LOGGING_FREQUENCY, false);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput)
    {
        this(value, type, isInput, TuningConstants.DEFAULT_LOGGING_FREQUENCY, false);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput, int loggingFrequency)
    {
        this(value, type, isInput, loggingFrequency, false);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput, boolean shouldLogToCsv)
    {
        this(value, type, isInput, TuningConstants.DEFAULT_LOGGING_FREQUENCY, shouldLogToCsv);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput, int loggingFrequency, boolean shouldLogToCsv)
    {
        if (loggingFrequency <= 0)
        {
            loggingFrequency = TuningConstants.DEFAULT_LOGGING_FREQUENCY;
        }

        this.value = value;
        this.type = type;
        this.isInput = isInput;
        this.loggingFrequency = loggingFrequency;
        this.shouldLogToCsv = shouldLogToCsv;
    }
}
