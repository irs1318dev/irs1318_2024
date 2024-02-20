package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.robotprovider.*;

@Singleton
public class SmartDashboardSelectionManager
{
    private final ISendableChooser<StartPosition> positionChooser;
    private final ISendableChooser<AutoRoutine> routineChooser;
    private ISendableChooser<PriorityPickupSide> pickupChooser;

    public enum StartPosition
    {
        NearAmp,
        SubwooferFront,
        SubwooferSide,
        NearSource,
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
        ThreeNote,
        TwoNote,
        AmpThreeNote,
    }

    public enum PriorityPickupSide
    {
        NearSubwoofer,
        MiddleAmp,
        MiddleSource,
    }


    /**
     * Initializes a new SmartDashboardSelectionManager
     */
    @Inject
    public SmartDashboardSelectionManager(
        IRobotProvider provider)
    {
        INetworkTableProvider networkTableProvider = provider.getNetworkTableProvider();

        this.routineChooser = networkTableProvider.getSendableChooser();
        this.routineChooser.addDefault("None", AutoRoutine.None);
        this.routineChooser.addObject("Shoot", AutoRoutine.Shoot);
        this.routineChooser.addObject("Taxi", AutoRoutine.Taxi);
        this.routineChooser.addObject("Shoot Taxi", AutoRoutine.ShootTaxi);
        this.routineChooser.addObject("Two Note", AutoRoutine.TwoNote);
        this.routineChooser.addObject("Three Note", AutoRoutine.ThreeNote);
        this.routineChooser.addObject("Four Note", AutoRoutine.FourNote);
        this.routineChooser.addObject("Amp Three Note", AutoRoutine.AmpThreeNote);
        this.routineChooser.addObject("One Plus Charge", AutoRoutine.FiveNote);
        this.routineChooser.addObject("One Pickup Charge", AutoRoutine.SixNote);
        networkTableProvider.addChooser("Auto Routine", this.routineChooser);

        this.positionChooser = networkTableProvider.getSendableChooser();
        this.positionChooser.addDefault("sub-front", StartPosition.SubwooferFront);
        this.positionChooser.addObject("sub-side", StartPosition.SubwooferSide);
        this.positionChooser.addObject("near-amp", StartPosition.NearAmp);
        this.positionChooser.addObject("near-source", StartPosition.NearSource);
        networkTableProvider.addChooser("Start Position", this.positionChooser);

        this.pickupChooser = networkTableProvider.getSendableChooser();
        this.pickupChooser.addDefault("Near Subwoofer", PriorityPickupSide.NearSubwoofer);
        this.pickupChooser.addObject("Mid Amp Side", PriorityPickupSide.MiddleAmp);
        this.pickupChooser.addObject("Mid Source Side", PriorityPickupSide.MiddleSource);
        networkTableProvider.addChooser("Start Position", this.positionChooser);        
    }

    public StartPosition getSelectedStartPosition()
    {

        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.positionChooser, StartPosition.SubwooferFront);
    }

    public AutoRoutine getSelectedAutoRoutine()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.routineChooser, AutoRoutine.None);
    }

    public PriorityPickupSide getPickupSide()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.pickupChooser, PriorityPickupSide.NearSubwoofer);
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
