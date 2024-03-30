package frc.robot.driver;

import java.util.EnumSet;

import javax.inject.Singleton;

import frc.lib.driver.*;
import frc.lib.driver.buttons.*;
import frc.lib.driver.descriptions.*;
import frc.lib.helpers.Helpers;
import frc.robot.*;
import frc.robot.driver.controltasks.*;
import frc.robot.driver.controltasks.FieldOrientationTask.DesiredOrientation;
import frc.robot.driver.controltasks.FollowPathTask.Type;
import frc.robot.driver.controltasks.VisionContinuousTurningTask.TurnType;
import frc.robot.driver.controltasks.VisionMoveAndTurnTaskBase.MoveType;

@Singleton
public class ButtonMap implements IButtonMap
{
    private static ShiftDescription[] ShiftButtonSchema = new ShiftDescription[]
    {
        new ShiftDescription(
            Shift.DriverDebug,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON),
        new ShiftDescription(
            Shift.CodriverDebug,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.Test1Debug,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.Test2Debug,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
    };

    public static AnalogOperationDescription[] AnalogOperationSchema = new AnalogOperationDescription[]
    {
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveForward,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSY,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_Y,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_Y,
            1.0,
            TuningConstants.SDSDRIVETRAIN_EXPONENTIAL),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSX,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            1.0,
            TuningConstants.SDSDRIVETRAIN_EXPONENTIAL),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurnAngleGoal,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RSX,
            AnalogAxis.XBONE_RSY,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            !ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS, // make left positive...
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            0.0,
            TuningConstants.SDSDRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA,
            true,
            TuningConstants.MAGIC_NULL_VALUE,
            (x, y) -> Helpers.atan2d(x, y)),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainSpinLeft,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LT,
            ElectronicsConstants.INVERT_XBONE_LEFT_TRIGGER, // turning left should be positive, as counter-clockwise is positive
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainSpinRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            !ElectronicsConstants.INVERT_XBONE_RIGHT_TRIGGER, // make left positive, as counter-clockwise is positive
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN),

        new AnalogOperationDescription(
            AnalogOperation.EndEffectorFlywheelMotorPower,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RT,
            ElectronicsConstants.INVERT_XBONE_RIGHT_TRIGGER,
            -1.0,
            0.2),

        new AnalogOperationDescription(
            AnalogOperation.EndEffectorGetNoteOut,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LT,
            ElectronicsConstants.INVERT_XBONE_LEFT_TRIGGER,
            -1.0,
            0.2),

        new AnalogOperationDescription(
            AnalogOperation.EndEffectorNearFlywheelVelocityGoal,
            TuningConstants.MAGIC_NULL_VALUE),

        new AnalogOperationDescription(
            AnalogOperation.EndEffectorFarFlywheelVelocityGoal,
            TuningConstants.MAGIC_NULL_VALUE),

        new AnalogOperationDescription(
            AnalogOperation.ArmShoulderPower,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LSY,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.ARM_SHOULDER_DEAD_ZONE,
            TuningConstants.ARM_SHOULDER_DEAD_ZONE,
            TuningConstants.ARM_SHOULDER_POWER_STRENGTH,
            TuningConstants.ARM_POWER_EXPONENTIAL),

        new AnalogOperationDescription(
            AnalogOperation.ArmShoulderAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LSY,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.ARM_SHOULDER_DEAD_ZONE,
            TuningConstants.ARM_SHOULDER_DEAD_ZONE,
            TuningConstants.ARM_SHOULDER_POWER_STRENGTH,
            TuningConstants.ARM_POWER_EXPONENTIAL),
        
        new AnalogOperationDescription(
            AnalogOperation.ArmWristPower,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RSY,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            -TuningConstants.ARM_WRIST_DEAD_ZONE,
            TuningConstants.ARM_WRIST_DEAD_ZONE,
            TuningConstants.ARM_WRIST_POWER_STRENGTH,
            TuningConstants.ARM_POWER_EXPONENTIAL),
        
        new AnalogOperationDescription(
            AnalogOperation.ArmWristAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RSY,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            -TuningConstants.ARM_WRIST_DEAD_ZONE,
            TuningConstants.ARM_WRIST_DEAD_ZONE,
            TuningConstants.ARM_WRIST_POWER_STRENGTH,
            TuningConstants.ARM_POWER_EXPONENTIAL),

        new AnalogOperationDescription(
            AnalogOperation.ArmShoulderPositionSetpoint,
            TuningConstants.MAGIC_NULL_VALUE),
            
        new AnalogOperationDescription(
            AnalogOperation.ArmWristPositionSetpoint,
            TuningConstants.MAGIC_NULL_VALUE),

        new AnalogOperationDescription(
            AnalogOperation.ArmAbsWristAngle,
            TuningConstants.MAGIC_NULL_VALUE),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        // driving operations
        new DigitalOperationDescription(
            DigitalOperation.PositionResetFieldOrientation,
            UserInputDevice.Driver,
            0, // DPAD-up
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Click),
        // new DigitalOperationDescription(
            // DigitalOperation.DriveTrainReset,
            // UserInputDevice.Driver,
            // UserInputDeviceButton.XBONE_Y_BUTTON,
            // EnumSet.of(Shift.DriverDebug),
            // EnumSet.noneOf(Shift.class),
            // ButtonType.Click),
        // new DigitalOperationDescription(
            // DigitalOperation.DriveTrainEnableFieldOrientation,
            // UserInputDevice.Driver,
            // 270,
            // EnumSet.of(Shift.DriverDebug),
            // EnumSet.noneOf(Shift.class),
            // ButtonType.Click),
        // new DigitalOperationDescription(
            // DigitalOperation.DriveTrainDisableFieldOrientation,
            // UserInputDevice.Driver,
            // 270,
            // EnumSet.of(Shift.DriverDebug),
            // EnumSet.of(Shift.DriverDebug),
            // ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.DriveTrainEnableMaintainDirectionMode,
        //     UserInputDevice.Driver,
        //     90,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.DriveTrainDisableMaintainDirectionMode,
        //     UserInputDevice.Driver,
        //     90,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainSlowMode,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.IntakeIn,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.IntakeOut,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.IntakeForceSpinOn,
            UserInputDevice.Codriver,
            0,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),

        new DigitalOperationDescription(
            DigitalOperation.IntakeForceSpinOff,
            UserInputDevice.Codriver,
            0,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Click),
    
        new DigitalOperationDescription(
            DigitalOperation.ShooterFeedRing,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            // EnumSet.of(Shift.DriverDebug),
            // EnumSet.noneOf(Shift.class),
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.ArmEnableSimpleMode,
            UserInputDevice.Codriver,
            180,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.ClimberWinchDown,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.ArmDisableSimpleMode,
            UserInputDevice.Codriver,
            180,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.ArmForceReset,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Click),

        // new DigitalOperationDescription(
        //     DigitalOperation.IntakeForceStop,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_RIGHT_BUTTON,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.ShooterShootAnywayMode,
            UserInputDevice.Codriver,
            90,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle),

        // new DigitalOperationDescription(
        //     DigitalOperation.ShooterDisableShootAnywayMode,
        //     UserInputDevice.Codriver,
        //     90,
        //     EnumSet.of(Shift.CodriverDebug),
        //     EnumSet.of(Shift.CodriverDebug),
        //     ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.ArmDisableThroughBore,
            UserInputDevice.Codriver, 
            UserInputDeviceButton.XBONE_SELECT_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Click),

        // new DigitalOperationDescription(
        //     DigitalOperation.ArmEnableThroughBore,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.XBONE_START_BUTTON,
        //     EnumSet.of(Shift.CodriverDebug),
        //     EnumSet.of(Shift.CodriverDebug),
        //     ButtonType.Click),

        new DigitalOperationDescription(
            DigitalOperation.ClimberServoUp,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_B_BUTTON,
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.ClimberServoDown,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            ButtonType.Simple),


        // Vision test operations:
        // new DigitalOperationDescription(
        //     DigitalOperation.VisionFindSpeakerAprilTagRear,
        //     UserInputDevice.Test1,
        //     UserInputDeviceButton.XBONE_A_BUTTON,
        //     EnumSet.of(Shift.Test1Debug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Simple),
        // new DigitalOperationDescription(
        //     DigitalOperation.VisionFindSpeakerAprilTagFront,
        //     UserInputDevice.Test1,
        //     UserInputDeviceButton.XBONE_A_BUTTON,
        //     EnumSet.of(Shift.Test1Debug),
        //     EnumSet.of(Shift.Test1Debug),
        //     ButtonType.Simple),
        // new DigitalOperationDescription(
        //     DigitalOperation.VisionFindStageAprilTagsRear,
        //     UserInputDevice.Test1,
        //     UserInputDeviceButton.XBONE_Y_BUTTON,
        //     EnumSet.of(Shift.Test1Debug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Simple),
        // new DigitalOperationDescription(
        //     DigitalOperation.VisionFindStageAprilTagsFront,
        //     UserInputDevice.Test1,
        //     UserInputDeviceButton.XBONE_Y_BUTTON,
        //     EnumSet.of(Shift.Test1Debug),
        //     EnumSet.of(Shift.Test1Debug),
        //     ButtonType.Simple),
        // new DigitalOperationDescription(
        //     DigitalOperation.VisionFindAmpAprilTagRear,
        //     UserInputDevice.Test1,
        //     UserInputDeviceButton.XBONE_X_BUTTON,
        //     EnumSet.of(Shift.Test1Debug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Simple),
        // new DigitalOperationDescription(
        //     DigitalOperation.VisionFindAmpAprilTagFront,
        //     UserInputDevice.Test1,
        //     UserInputDeviceButton.XBONE_X_BUTTON,
        //     EnumSet.of(Shift.Test1Debug),
        //     EnumSet.of(Shift.Test1Debug),
        //     ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.VisionFindAnyAprilTagRear,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.VisionFindAnyAprilTagFront,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.of(Shift.Test1Debug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.VisionFindAbsolutePosition,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_X_BUTTON,
            ButtonType.Simple),
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // driving macros
        // new MacroOperationDescription(
        //     MacroOperation.PIDLightBrake,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Simple,
        //     () -> new PIDBrakeTask(false),
        //     new IOperation[]
        //     {
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //     }),
        new MacroOperationDescription(
            MacroOperation.PIDHeavyBrake,
            UserInputDevice.Driver,
            90, // DPAD-right
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple,
            () -> new PIDBrakeTask(true),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
            }),

        new MacroOperationDescription(
            MacroOperation.FaceForward,
            UserInputDevice.Driver,
            0, // DPAD-up
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new OrientationTask(0),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
            }),
        

        new MacroOperationDescription(
            MacroOperation.FaceLeft,
            UserInputDevice.Driver,
            270, // DPAD-left
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new OrientationTask(60),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
            }),

        new MacroOperationDescription(
            MacroOperation.AlignStage,
            UserInputDevice.Driver,
            270, // DPAD-left
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Toggle,
            () -> new VisionApproachAprilTagTask(36.0, 0.0, DigitalOperation.VisionFindStageAprilTagsFront),
            new IOperation[]
            {
                DigitalOperation.VisionFindAnyAprilTagFront,
                DigitalOperation.VisionFindAnyAprilTagRear,
                DigitalOperation.VisionFindSpeakerAprilTagFront,
                DigitalOperation.VisionFindSpeakerAprilTagRear,
                DigitalOperation.VisionFindAmpAprilTagFront,
                DigitalOperation.VisionFindAmpAprilTagRear,
                DigitalOperation.VisionFindStageAprilTagsFront,
                DigitalOperation.VisionFindStageAprilTagsRear,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.DriveTrainResetXYPosition,
                DigitalOperation.DriveTrainEnableMaintainDirectionMode,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
            }),

        new MacroOperationDescription(
            MacroOperation.FaceRight,
            UserInputDevice.Driver,
            90, // DPAD-right
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new OrientationTask(-60),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
            }),

        new MacroOperationDescription(
            MacroOperation.FaceBackward,
            UserInputDevice.Driver,
            180, // DPAD-down
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FieldOrientationTask(DesiredOrientation.Amp),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
            }),

        new MacroOperationDescription(
            MacroOperation.AlignAmp,
            UserInputDevice.Driver,
            180, // DPAD-down
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new FieldOrientationTask(DesiredOrientation.Amp),
                new VisionApproachAprilTagTask(40.0, 0.0, DigitalOperation.VisionFindAmpAprilTagFront),
                new VisionApproachAprilTagTask(20.0, 0.0, DigitalOperation.VisionFindAmpAprilTagFront),
                new FieldOrientationTask(DesiredOrientation.Amp)),
            new IOperation[]
            {
                DigitalOperation.VisionFindAnyAprilTagFront,
                DigitalOperation.VisionFindAnyAprilTagRear,
                DigitalOperation.VisionFindSpeakerAprilTagFront,
                DigitalOperation.VisionFindSpeakerAprilTagRear,
                DigitalOperation.VisionFindAmpAprilTagFront,
                DigitalOperation.VisionFindAmpAprilTagRear,
                DigitalOperation.VisionFindStageAprilTagsFront,
                DigitalOperation.VisionFindStageAprilTagsRear,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.DriveTrainResetXYPosition,
                DigitalOperation.DriveTrainEnableMaintainDirectionMode,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest1,
            UserInputDevice.Test1,
            0,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new FollowPathTask("goForwards30in", Type.RobotRelativeFromCurrentPose)
            ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                DigitalOperation.DriveTrainEnableMaintainDirectionMode,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindSpeakerAprilTagRear,
                DigitalOperation.VisionFindSpeakerAprilTagFront,
                DigitalOperation.VisionFindAnyAprilTagRear,
                DigitalOperation.VisionFindAnyAprilTagFront,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),

        // Full auton test
        new MacroOperationDescription(
            MacroOperation.FollowPathTest2,
            UserInputDevice.Test1,
            180,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FollowPathTask("goLeft30in", Type.RobotRelativeFromCurrentPose),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindSpeakerAprilTagRear,
                DigitalOperation.VisionFindSpeakerAprilTagFront,
                DigitalOperation.VisionFindAnyAprilTagRear,
                DigitalOperation.VisionFindAnyAprilTagFront,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest3,
            UserInputDevice.Test1,
            270,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FollowPathTask("goLeft22in", Type.RobotRelativeFromCurrentPose),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindSpeakerAprilTagRear,
                DigitalOperation.VisionFindSpeakerAprilTagFront,
                DigitalOperation.VisionFindAnyAprilTagRear,
                DigitalOperation.VisionFindAnyAprilTagFront,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest4,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () ->
                new ForwardsUntilNoteTask(),
            
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindSpeakerAprilTagRear,
                DigitalOperation.VisionFindSpeakerAprilTagFront,
                DigitalOperation.VisionFindAnyAprilTagRear,
                DigitalOperation.VisionFindAnyAprilTagFront,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),

            /*

        new MacroOperationDescription(
            MacroOperation.ShootAuto,
            UserInputDevice.Test2,
            90,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequenceTask.Sequence(
                new ArmZeroTask(),
                new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
                ConcurrentTask.AnyTasks(
                    new ShooterSpinTask(3800, 10.0),
                    SequentialTask.Sequence(
                        new WaitTask(3.0),
                        new FeedRingTask(true, 2.0))
            ))),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindSpeakerAprilTagRear,
                DigitalOperation.VisionFindSpeakerAprilTagFront,
                DigitalOperation.VisionFindAnyAprilTagRear,
                DigitalOperation.VisionFindAnyAprilTagFront,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),

            */

        new MacroOperationDescription(
            MacroOperation.ResetArm,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new ArmZeroTask(),
            new IOperation[]
            {
                AnalogOperation.ArmShoulderPower,
                AnalogOperation.ArmWristPower,
                AnalogOperation.ArmShoulderPositionSetpoint,
                AnalogOperation.ArmWristPositionSetpoint,
                DigitalOperation.ArmForceReset,
                DigitalOperation.ArmStop,
            }),

        new MacroOperationDescription(
            MacroOperation.ArmShoulderWristPosition1,
            UserInputDevice.Codriver, 
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle, 
            () -> new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION),
            new IOperation[]
            {
                AnalogOperation.ArmShoulderPositionSetpoint,
                AnalogOperation.ArmWristPositionSetpoint,
                AnalogOperation.ArmShoulderAdjustment,
                AnalogOperation.ArmWristAdjustment,
                AnalogOperation.ArmShoulderPower,
                AnalogOperation.ArmWristPower,
            }),

        // new MacroOperationDescription(
        // MacroOperation.ArmShoulderWristPosition1,
        // UserInputDevice.Test2, 
        // 90,
        // EnumSet.of(Shift.CodriverDebug),
        // EnumSet.noneOf(Shift.class),
        // ButtonType.Toggle, 
        // () -> new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_STARTING_CONFIGURATION),
        // new IOperation[]
        // {
        //     DigitalOperation.PositionResetFieldOrientation,
        //         DigitalOperation.PositionResetRobotLevel,
        //         AnalogOperation.PositionStartingAngle,
        //         DigitalOperation.DriveTrainResetXYPosition,
        //         AnalogOperation.DriveTrainStartingXPosition,
        //         AnalogOperation.DriveTrainStartingYPosition,
        //         AnalogOperation.DriveTrainMoveForward,
        //         AnalogOperation.DriveTrainMoveRight,
        //         AnalogOperation.DriveTrainTurnAngleGoal,
        //         AnalogOperation.DriveTrainSpinLeft,
        //         AnalogOperation.DriveTrainSpinRight,
        //         AnalogOperation.DriveTrainRotationA,
        //         AnalogOperation.DriveTrainRotationB,
        //         AnalogOperation.DriveTrainPathXGoal,
        //         AnalogOperation.DriveTrainPathYGoal,
        //         AnalogOperation.DriveTrainPathXVelocityGoal,
        //         AnalogOperation.DriveTrainPathYVelocityGoal,
        //         AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //         AnalogOperation.DriveTrainPositionDrive1,
        //         AnalogOperation.DriveTrainPositionDrive2,
        //         AnalogOperation.DriveTrainPositionDrive3,
        //         AnalogOperation.DriveTrainPositionDrive4,
        //         AnalogOperation.DriveTrainPositionSteer1,
        //         AnalogOperation.DriveTrainPositionSteer2,
        //         AnalogOperation.DriveTrainPositionSteer3,
        //         AnalogOperation.DriveTrainPositionSteer4,
        //         DigitalOperation.DriveTrainSteerMode,
        //         DigitalOperation.DriveTrainMaintainPositionMode,
        //         DigitalOperation.DriveTrainPathMode,
        //         DigitalOperation.DriveTrainReset,
        //         DigitalOperation.DriveTrainEnableFieldOrientation,
        //         DigitalOperation.DriveTrainDisableFieldOrientation,
        //         DigitalOperation.DriveTrainUseRobotOrientation,
        //         DigitalOperation.VisionEnableStream,
        //         DigitalOperation.VisionFindSpeakerAprilTagRear,
        //         DigitalOperation.VisionFindSpeakerAprilTagFront,
        //         DigitalOperation.VisionFindAnyAprilTagRear,
        //         DigitalOperation.VisionFindAnyAprilTagFront,
        //         DigitalOperation.VisionFindAbsolutePosition,
        //         DigitalOperation.VisionForceDisable,
        // }),

        // new MacroOperationDescription(
        //     MacroOperation.ArmShoulderWristPosition1,
        //     UserInputDevice.Codriver, 
        //     UserInputDeviceButton.XBONE_X_BUTTON,
        //     EnumSet.of(Shift.CodriverDebug),
        //     EnumSet.noneOf(Shift.class),
        //     ButtonType.Toggle, 
        //     () -> new ArmShoulderPositionTask(TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL),
        //     new IOperation[]
        //     {
        //         AnalogOperation.ArmShoulderPositionSetpoint,
        //         AnalogOperation.ArmWristPositionSetpoint,
                // AnalogOperation.ArmShoulderAdjustment,
                // AnalogOperation.ArmWristAdjustment,
                // AnalogOperation.ArmShoulderPower,
                // AnalogOperation.ArmWristPower,
        //     }),

        new MacroOperationDescription(
            MacroOperation.IntakeFix,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_START_BUTTON,
            // EnumSet.of(Shift.CodriverDebug),
            // EnumSet.of(Shift.CodriverDebug),
            ButtonType.Toggle,
            () -> new IntakeFixerTask(),
            new IOperation[]
            {
                DigitalOperation.IntakeOutSlow,
                DigitalOperation.IntakeOut,
                DigitalOperation.IntakeIn,
            }),


        new MacroOperationDescription(
           MacroOperation.ArmShoulderWristPosition2,
           UserInputDevice.Codriver, 
           UserInputDeviceButton.XBONE_Y_BUTTON,
           EnumSet.of(Shift.CodriverDebug),
           EnumSet.noneOf(Shift.class),
           ButtonType.Toggle, 
           () -> new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_PICKUP),
           new IOperation[]
           {
                AnalogOperation.ArmShoulderPositionSetpoint,
                AnalogOperation.ArmWristPositionSetpoint,
                AnalogOperation.ArmShoulderAdjustment,
                AnalogOperation.ArmWristAdjustment,
                AnalogOperation.ArmShoulderPower,
                AnalogOperation.ArmWristPower,
           }),

        new MacroOperationDescription(
            MacroOperation.ArmShoulderWristPosition3,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Toggle, 
            () -> new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_LOWER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_GROUND_SHOT),
            new IOperation[]
            {
                AnalogOperation.ArmShoulderPositionSetpoint,
                AnalogOperation.ArmWristPositionSetpoint,
                AnalogOperation.ArmShoulderAdjustment,
                AnalogOperation.ArmWristAdjustment,
                AnalogOperation.ArmShoulderPower,
                AnalogOperation.ArmWristPower,
            }),

        new MacroOperationDescription(
           MacroOperation.ArmShoulderWristPosition4,
           UserInputDevice.Codriver,
           UserInputDeviceButton.XBONE_B_BUTTON,
           EnumSet.of(Shift.CodriverDebug),
           EnumSet.noneOf(Shift.class),
           ButtonType.Toggle, 
           () -> new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_UPPER_UNIVERSAL_SHOT),
           new IOperation[]
           {
                AnalogOperation.ArmShoulderPositionSetpoint,
                AnalogOperation.ArmWristPositionSetpoint,
                AnalogOperation.ArmShoulderAdjustment,
                AnalogOperation.ArmWristAdjustment,
                AnalogOperation.ArmShoulderPower,
                AnalogOperation.ArmWristPower,
           }),

        // new MacroOperationDescription(
            // MacroOperation.ArmShoulderWristPosition5,
            // UserInputDevice.Codriver, 
            // UserInputDeviceButton.XBONE_B_BUTTON,
            // EnumSet.of(Shift.CodriverDebug),
            // EnumSet.of(Shift.CodriverDebug),
            // ButtonType.Toggle, 
            // () -> new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_TUCKED, TuningConstants.ARM_WRIST_POSITION_TUCKED_SHOT),
            // new IOperation[]
            // {
                // AnalogOperation.ArmShoulderPositionSetpoint,
                // AnalogOperation.ArmWristPositionSetpoint,
                // AnalogOperation.ArmShoulderAdjustment,
                // AnalogOperation.ArmWristAdjustment,
                // AnalogOperation.ArmShoulderPower,
                // AnalogOperation.ArmWristPower,
            // }),

        // new MacroOperationDescription(
        //    MacroOperation.ArmShoulderWristPosition6,
        //    UserInputDevice.Codriver, 
        //    UserInputDeviceButton.XBONE_A_BUTTON,
        //    EnumSet.of(Shift.CodriverDebug),
        //    EnumSet.noneOf(Shift.class),
        //    ButtonType.Toggle, 
        //    () -> new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_AMP_SCORE, TuningConstants.ARM_WRIST_POSITION_AMP_SCORE),
        //    new IOperation[]
        //    {
                // AnalogOperation.ArmShoulderPositionSetpoint,
                // AnalogOperation.ArmWristPositionSetpoint,
                // AnalogOperation.ArmShoulderAdjustment,
                // AnalogOperation.ArmWristAdjustment,
                // AnalogOperation.ArmShoulderPower,
                // AnalogOperation.ArmWristPower,
        //    }),

        new MacroOperationDescription(
            MacroOperation.ArmShoulderWristPosition7,
            UserInputDevice.Codriver, 
            UserInputDeviceButton.XBONE_A_BUTTON,
            // EnumSet.of(Shift.CodriverDebug),
            // EnumSet.of(Shift.CodriverDebug),
            ButtonType.Toggle, 
            () -> new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_AMP_OUTTAKE, TuningConstants.ARM_WRIST_POSITION_AMP_OUTTAKE),
            new IOperation[]
            {
                AnalogOperation.ArmShoulderPositionSetpoint,
                AnalogOperation.ArmWristPositionSetpoint,
                AnalogOperation.ArmShoulderAdjustment,
                AnalogOperation.ArmWristAdjustment,
                AnalogOperation.ArmShoulderPower,
                AnalogOperation.ArmWristPower,
            }),

        new MacroOperationDescription(
            MacroOperation.ArmShoulderWristPosition8,
            UserInputDevice.Codriver, 
            UserInputDeviceButton.XBONE_SELECT_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle, 
            () -> new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_TRAP_INTERMEDIATE, TuningConstants.ARM_WRIST_POSITION_TRAP_INTERMEDIATE),
            new IOperation[]
            {
                AnalogOperation.ArmShoulderPositionSetpoint,
                AnalogOperation.ArmWristPositionSetpoint,
                AnalogOperation.ArmShoulderAdjustment,
                AnalogOperation.ArmWristAdjustment,
                AnalogOperation.ArmShoulderPower,
                AnalogOperation.ArmWristPower,
            }),

        new MacroOperationDescription(
            MacroOperation.ArmShoulderWristPosition9,
            UserInputDevice.Codriver, 
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Toggle, 
            () -> new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_QUICK_TUCK, TuningConstants.ARM_WRIST_POSITION_QUICK_TUCK),
            new IOperation[]
            {
                AnalogOperation.ArmShoulderPositionSetpoint,
                AnalogOperation.ArmWristPositionSetpoint,
                AnalogOperation.ArmShoulderAdjustment,
                AnalogOperation.ArmWristAdjustment,
                AnalogOperation.ArmShoulderPower,
                AnalogOperation.ArmWristPower,
            }),

        new MacroOperationDescription(
            MacroOperation.ArmShoulderWristPosition10,
            UserInputDevice.Codriver, 
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Toggle, 
            () -> new ArmGraphTask(TuningConstants.ARM_SHOULDER_POSITION_UPPER_UNIVERSAL, TuningConstants.ARM_WRIST_POSITION_BOT_DISTANCE_SHOT),
            new IOperation[]
            {
                AnalogOperation.ArmShoulderPositionSetpoint,
                AnalogOperation.ArmWristPositionSetpoint,
                AnalogOperation.ArmShoulderAdjustment,
                AnalogOperation.ArmWristAdjustment,
                AnalogOperation.ArmShoulderPower,
                AnalogOperation.ArmWristPower,
            }),
// 
        // new MacroOperationDescription(
        //    MacroOperation.ArmWristPosition4,
        //    UserInputDevice.Codriver, 
        //    UserInputDeviceButton.XBONE_Y_BUTTON,
        //    EnumSet.of(Shift.CodriverDebug),
        //    EnumSet.of(Shift.CodriverDebug),
        //    ButtonType.Toggle, 
        //    () -> new ArmWristPositionTask(TuningConstants.ARM_WRIST_POSITION_AMP_SCORE),
        //    new IOperation[]
        //    {
            //    AnalogOperation.ArmWristPositionSetpoint,
        //    }),
         
    //    new MacroOperationDescription(
            // MacroOperation.ArmWristPosition5,
            // UserInputDevice.Codriver, 
            // UserInputDeviceButton.XBONE_SELECT_BUTTON,
            // EnumSet.of(Shift.CodriverDebug),
            // EnumSet.noneOf(Shift.class),
            // ButtonType.Toggle, 
            // () -> new ArmWristPositionTask(TuningConstants.ARM_WRIST_POSITION_TUCKED_SHOT),
            // new IOperation[]
            // {
                // AnalogOperation.ArmWristPositionSetpoint,
            // }),
    
        new MacroOperationDescription(
            MacroOperation.ShooterSpinSpeed1,
            UserInputDevice.Codriver, 
            270,
            // EnumSet.of(Shift.CodriverDebug),
            // EnumSet.noneOf(Shift.class),
            ButtonType.Simple,
            () -> new ShooterSpinTask(3800.0),
            new IOperation[]
            {
                AnalogOperation.EndEffectorFarFlywheelVelocityGoal,
                AnalogOperation.EndEffectorNearFlywheelVelocityGoal,
                AnalogOperation.EndEffectorFlywheelMotorPower,
                AnalogOperation.EndEffectorGetNoteOut,
            }),
            
        // new MacroOperationDescription(
            // MacroOperation.ShooterSpinSpeed2,
            // UserInputDevice.Codriver, 
            // 270,
            // EnumSet.of(Shift.CodriverDebug),
            // EnumSet.of(Shift.CodriverDebug),
            // ButtonType.Simple,
            // () -> new ShooterSpinTask(3200.0), // 3500.0
            // new IOperation[]
            // {
                // AnalogOperation.EndEffectorFarFlywheelVelocityGoal,
                // AnalogOperation.EndEffectorNearFlywheelVelocityGoal,
                // AnalogOperation.EndEffectorFlywheelMotorPower,
                // AnalogOperation.EndEffectorGetNoteOut,
            // }),

        new MacroOperationDescription(
            MacroOperation.ShootNote,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_RIGHT_STICK_BUTTON,
            ButtonType.Toggle,
            () -> new VisionShootNoteFullTask(),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.VisionFindSpeakerAprilTagRear,
                DigitalOperation.VisionFindSpeakerAprilTagFront,
                DigitalOperation.VisionFindAnyAprilTagRear,
                DigitalOperation.VisionFindAnyAprilTagFront,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.IntakeIn,
                DigitalOperation.IntakeOut,
                DigitalOperation.ShooterFeedRing,
                AnalogOperation.EndEffectorFlywheelMotorPower,
                AnalogOperation.EndEffectorNearFlywheelVelocityGoal,
                AnalogOperation.EndEffectorFarFlywheelVelocityGoal,
                AnalogOperation.EndEffectorGetNoteOut,
            }),

        // new MacroOperationDescription(
        //     MacroOperation.ClimbMacro,
        //     UserInputDevice.Codriver, 
        //     UserInputDeviceButton.XBONE_A_BUTTON,
        //     ButtonType.Toggle, 
        //     () -> SequentialTask(
        //         new OrientationTask(180),
        //         new VisionAprilTagTranslateTask()),
        //     new IOperation[]
        //     {
        //         AnalogOperation.ClimberPower,
        //     }),

        new MacroOperationDescription(
            MacroOperation.VisionShootAim,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> VisionShooterTurnAndAimAbsoluteTask.createShootMacroTask(),
            new IOperation[]
            {
                AnalogOperation.ArmWristPositionSetpoint,
                AnalogOperation.ArmShoulderPositionSetpoint,
                AnalogOperation.ArmAbsWristAngle,
                AnalogOperation.EndEffectorNearFlywheelVelocityGoal,
                AnalogOperation.EndEffectorFarFlywheelVelocityGoal,
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                // AnalogOperation.DriveTrainMoveForward,
                // AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindSpeakerAprilTagRear,
                DigitalOperation.VisionFindSpeakerAprilTagFront,
                DigitalOperation.VisionFindStageAprilTagsRear,
                DigitalOperation.VisionFindStageAprilTagsFront,
                DigitalOperation.VisionFindAmpAprilTagRear,
                DigitalOperation.VisionFindAmpAprilTagFront,
                DigitalOperation.VisionFindAnyAprilTagRear,
                DigitalOperation.VisionFindAnyAprilTagFront,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.ForceLightDriverRumble,
            }),

        new MacroOperationDescription(
            MacroOperation.VisionAbsoluteShootAim,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Toggle,
            () -> VisionShooterTurnAndAimAbsoluteTask.createShootMacroTask(),
            new IOperation[]
            {
                AnalogOperation.ArmWristPositionSetpoint,
                AnalogOperation.ArmShoulderPositionSetpoint,
                AnalogOperation.ArmAbsWristAngle,
                AnalogOperation.EndEffectorNearFlywheelVelocityGoal,
                AnalogOperation.EndEffectorFarFlywheelVelocityGoal,
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                // AnalogOperation.DriveTrainMoveForward,
                // AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindSpeakerAprilTagRear,
                DigitalOperation.VisionFindSpeakerAprilTagFront,
                DigitalOperation.VisionFindStageAprilTagsRear,
                DigitalOperation.VisionFindStageAprilTagsFront,
                DigitalOperation.VisionFindAmpAprilTagRear,
                DigitalOperation.VisionFindAmpAprilTagFront,
                DigitalOperation.VisionFindAnyAprilTagRear,
                DigitalOperation.VisionFindAnyAprilTagFront,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.ForceLightDriverRumble,
            }),

        new MacroOperationDescription(
            MacroOperation.AutoTrapScore,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ShootTrapTask(TuningConstants.TRAP_DRIVE_SHOT_X_OFFSET, 0, DigitalOperation.VisionFindStageAprilTagsFront)
            ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindSpeakerAprilTagRear,
                DigitalOperation.VisionFindSpeakerAprilTagFront,
                DigitalOperation.VisionFindAnyAprilTagRear,
                DigitalOperation.VisionFindAnyAprilTagFront,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),


        // new MacroOperationDescription(
        // MacroOperation.ClimbMacro, //approach chain and climb
        // UserInputDevice.Test1,
        // UserInputDeviceButton.XBONE_X_BUTTON,
        // EnumSet.noneOf(Shift.class),
        // EnumSet.noneOf(Shift.class),
        // ButtonType.Toggle,
        // () -> SequentialTask.Sequence(
        //     new ApproachAprilTagTask(24.0, 0.0, DigitalOperation.VisionFindStageAprilTagsFront),
        //     new ClimberWinchTask(ClimberWinchTask.WinchState.Retracted)
        // ),
        // new IOperation[]
        // {
        //     DigitalOperation.PositionResetFieldOrientation,
        //     DigitalOperation.PositionResetRobotLevel,
        //     AnalogOperation.PositionStartingAngle,
        //     DigitalOperation.DriveTrainResetXYPosition,
        //     AnalogOperation.DriveTrainStartingXPosition,
        //     AnalogOperation.DriveTrainStartingYPosition,
        //     AnalogOperation.DriveTrainMoveForward,
        //     AnalogOperation.DriveTrainMoveRight,
        //     AnalogOperation.DriveTrainTurnAngleGoal,
        //     AnalogOperation.DriveTrainSpinLeft,
        //     AnalogOperation.DriveTrainSpinRight,
        //     AnalogOperation.DriveTrainRotationA,
        //     AnalogOperation.DriveTrainRotationB,
        //     AnalogOperation.DriveTrainPathXGoal,
        //     AnalogOperation.DriveTrainPathYGoal,
        //     AnalogOperation.DriveTrainPathXVelocityGoal,
        //     AnalogOperation.DriveTrainPathYVelocityGoal,
        //     AnalogOperation.DriveTrainPathAngleVelocityGoal,
        //     AnalogOperation.DriveTrainPositionDrive1,
        //     AnalogOperation.DriveTrainPositionDrive2,
        //     AnalogOperation.DriveTrainPositionDrive3,
        //     AnalogOperation.DriveTrainPositionDrive4,
        //     AnalogOperation.DriveTrainPositionSteer1,
        //     AnalogOperation.DriveTrainPositionSteer2,
        //     AnalogOperation.DriveTrainPositionSteer3,
        //     AnalogOperation.DriveTrainPositionSteer4,
        //     DigitalOperation.DriveTrainSteerMode,
        //     DigitalOperation.DriveTrainMaintainPositionMode,
        //     DigitalOperation.DriveTrainPathMode,
        //     DigitalOperation.DriveTrainReset,
        //     DigitalOperation.DriveTrainEnableFieldOrientation,
        //     DigitalOperation.DriveTrainDisableFieldOrientation,
        //     DigitalOperation.DriveTrainUseRobotOrientation,
        //     DigitalOperation.VisionEnableStream,
        //     DigitalOperation.VisionFindSpeakerAprilTagRear,
        //     DigitalOperation.VisionFindSpeakerAprilTagFront,
        //     DigitalOperation.VisionFindAnyAprilTagRear,
        //     DigitalOperation.VisionFindAnyAprilTagFront,
        //     DigitalOperation.VisionFindAbsolutePosition,
        //     DigitalOperation.VisionForceDisable,
        // }),
    };

    @Override
    public ShiftDescription[] getShiftSchema()
    {
        return ButtonMap.ShiftButtonSchema;
    }

    @Override
    public AnalogOperationDescription[] getAnalogOperationSchema()
    {
        return ButtonMap.AnalogOperationSchema;
    }

    @Override
    public DigitalOperationDescription[] getDigitalOperationSchema()
    {
        return ButtonMap.DigitalOperationSchema;
    }

    @Override
    public MacroOperationDescription[] getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}
