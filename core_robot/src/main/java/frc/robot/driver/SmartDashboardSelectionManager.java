package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.robotprovider.*;

@Singleton
public class SmartDashboardSelectionManager
{
    private final ISendableChooser<StartPosition> positionChooser;
    private final ISendableChooser<AutoRoutine> routineChooser;
    private final ISendableChooser<PriorityPickupSide> pickupChooser;
    private final ISendableChooser<Object> wristSlopChooser;
    private final IDoubleSubscriber wristSlopSlider;

    public enum StartPosition
    {
        None,
        Amp,
        WooferFront,
        WooferAmpSide,
        WooferSourceSide,
        Source,
    }

    public enum AutoRoutine
    {
        None,
        Shoot,
        Taxi,
        ShootTaxi,
        SixNote,
        FiveNote,
        FourNote,
        ThreePickupNote,
        ThreeNote,
        TwoNote,
        OneNote,
        AmpThreeNote,
    }

    public enum PriorityPickupSide
    {
        None,
        Close,
        Center,
    }

    public enum YesOrNo
    {
        No,
        Yes
    }

    /**
     * Initializes a new SmartDashboardSelectionManager
     */
    @Inject
    public SmartDashboardSelectionManager(
        IRobotProvider provider)
    {
        INetworkTableProvider networkTableProvider = provider.getNetworkTableProvider();

        this.routineChooser = networkTableProvider.getSendableChooser("Auto Routine");
        this.routineChooser.addDefault("None", AutoRoutine.None);
        this.routineChooser.addObject("Shoot", AutoRoutine.Shoot);
        this.routineChooser.addObject("Taxi", AutoRoutine.Taxi);
        this.routineChooser.addObject("Shoot Taxi", AutoRoutine.ShootTaxi);
        this.routineChooser.addObject("Two Note", AutoRoutine.TwoNote);
        this.routineChooser.addObject("Three Note", AutoRoutine.ThreeNote);
        this.routineChooser.addObject("Three Pickup Note", AutoRoutine.ThreePickupNote);
        this.routineChooser.addObject("Four Note", AutoRoutine.FourNote);
        this.routineChooser.addObject("Amp Three Note", AutoRoutine.AmpThreeNote);
        this.routineChooser.addObject("One Plus Charge", AutoRoutine.FiveNote);
        this.routineChooser.addObject("One Pickup Charge", AutoRoutine.SixNote);
        this.routineChooser.addObject("One Note", AutoRoutine.OneNote);

        this.positionChooser = networkTableProvider.getSendableChooser("Start Position");
        this.positionChooser.addDefault("None", StartPosition.None);
        this.positionChooser.addObject("sub-front", StartPosition.WooferFront);
        this.positionChooser.addObject("sub-source-side", StartPosition.WooferSourceSide);
        this.positionChooser.addObject("sub-amp-side", StartPosition.WooferAmpSide);
        this.positionChooser.addObject("near-amp", StartPosition.Amp);
        this.positionChooser.addObject("near-source", StartPosition.Source);

        this.pickupChooser = networkTableProvider.getSendableChooser("Pickup Chooser");
        this.pickupChooser.addDefault("None", PriorityPickupSide.None);
        this.pickupChooser.addObject("Near Subwoofer", PriorityPickupSide.Close);
        this.pickupChooser.addObject("Middle", PriorityPickupSide.Center);

        this.wristSlopChooser = networkTableProvider.getSendableChooser("Use WristSlop");
        this.wristSlopChooser.addDefault("No", YesOrNo.No);
        this.wristSlopChooser.addObject("Yes", YesOrNo.Yes);

        this.wristSlopSlider = networkTableProvider.getNumberSlider("WristSlopAdj", 0.0);
    }

    public StartPosition getSelectedStartPosition()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.positionChooser, StartPosition.WooferFront);
    }

    public AutoRoutine getSelectedAutoRoutine()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.routineChooser, AutoRoutine.None);
    }

    public PriorityPickupSide getPickupSide()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.pickupChooser, PriorityPickupSide.Close);
    }

    public boolean getUseWristSlop()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.wristSlopChooser, YesOrNo.No) == YesOrNo.Yes;
    }

    public double getWristSlopAdjustment()
    {
        return this.wristSlopSlider.get();
    }

    private static <T> T GetSelectedOrDefault(ISendableChooser<T> chooser, T defaultValue)
    {
        T selected = chooser.getSelected();
        if (selected == null)
        {
            selected = defaultValue;
        }

        return selected;
    }
}
