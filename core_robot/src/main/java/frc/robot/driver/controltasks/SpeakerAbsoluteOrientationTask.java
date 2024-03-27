package frc.robot.driver.controltasks;

import frc.lib.driver.IDriver;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.IRobotProvider;
import frc.robot.AutonLocManager;
import frc.robot.TuningConstants;
import frc.robot.mechanisms.OffboardVisionManager;
import frc.robot.mechanisms.PigeonManager;

public class SpeakerAbsoluteOrientationTask extends PIDTurnTaskBase
{
    private double absoluteXSpeakerWResToCenter;
    private double absoluteYSpeakerWResToCenter;
    private double absoluteXRobotWResToCenter;
    private double absoluteYRobotWResToCenter;
    private double orientationTheta;
    
    private boolean isRedAlliance;
    
    private PigeonManager pigeonManager;
    private IDriver driver;
    private LoggingManager loggingManager;
    private OffboardVisionManager visionManager;
    private IRobotProvider provider;
    private AutonLocManager manager;


    /**
     * 
     */
    public SpeakerAbsoluteOrientationTask(
        boolean bestEffort,
        boolean useTime)
    {
        super(useTime,bestEffort);
        //parameter initializations
        //to determine position of the speaker depending on the side
        this.absoluteXSpeakerWResToCenter = TuningConstants.CENTER_TO_SPEAKER_X_DISTANCE;
        this.absoluteYSpeakerWResToCenter = TuningConstants.CENTER_TO_SPEAKER_Y_DISTANCE;
    }


    @Override
    public void begin()
    {
        super.begin();

        

        this.driver = this.getInjector().getInstance(IDriver.class);
        this.provider = this.getInjector().getInstance(IRobotProvider.class);
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        this.pigeonManager = this.getInjector().getInstance(PigeonManager.class);
        this.manager = new AutonLocManager(this.provider);

        this.pigeonManager = new PigeonManager(this.driver,this.loggingManager,this.provider);
        this.isRedAlliance = manager.getRedUpdateAlliance();

    }


    @Override
    protected Double getHorizontalAngle()
    {
        // TODO Auto-generated method stub
        this.absoluteXRobotWResToCenter = this.visionManager.getAbsolutePositionX();
        this.absoluteYRobotWResToCenter = this.visionManager.getAbsolutePositionY();

        double absoluteDeltaXSpeakerToRobot = this.absoluteXRobotWResToCenter - this.absoluteXSpeakerWResToCenter;
        double absoluteDeltaYSpeakerToRobot = this.absoluteYRobotWResToCenter - this.absoluteYSpeakerWResToCenter;
        
        this.orientationTheta = this.pigeonManager.getYaw() - Helpers.atan2d(
            absoluteDeltaXSpeakerToRobot,
            absoluteDeltaYSpeakerToRobot);

        return (this.isRedAlliance) ? this.orientationTheta : - 1 * this.orientationTheta;
    }
}
