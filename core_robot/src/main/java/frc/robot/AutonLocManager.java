package frc.robot;
import java.util.Optional;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.driver.TrajectoryManager;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IDriverStation;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.Point2d;

public class AutonLocManager {

    boolean isRed;
    IRobotProvider provider;

    public AutonLocManager(boolean isRed)
    {
        this.isRed = isRed;
        this.provider = null;
    }
    
    public AutonLocManager(IRobotProvider provider) 
    {
        this.provider = provider;
    }

    public void updateAlliance()
    {
        IDriverStation driverStation = provider.getDriverStation();
        Optional<Alliance> alliance = driverStation.getAlliance();
        isRed = alliance.isPresent() || alliance.get() == Alliance.Red;
    }

    //Red is positive
    //Blue is negative

    //Y field length: 323in
    //X field length: 653in

    public double framePreremetere = 34; //With bumpers
    public double halfFramePreremetere = framePreremetere / 2.0;

    public Point2d P1 = new Point2d(getXPosition(isRed, 326.5 - halfFramePreremetere), 64 + halfFramePreremetere);
    public Point2d P2 = new Point2d(getXPosition(isRed, 289 - 21.22) , 198); //210 degrees orientation
    public Point2d P3 = new Point2d(getXPosition(isRed, 250.5 + halfFramePreremetere), 306 - halfFramePreremetere);

    public Point2d P4 = new Point2d(getXPosition(isRed, 288 - halfFramePreremetere), 239 - halfFramePreremetere);

    public Point2d P5 = new Point2d(getXPosition(isRed, 212), 162);
    public Point2d P5M = new Point2d(getXPosition(isRed, P5.x + 20), P5.y);

    public Point2d P6 = new Point2d(getXPosition(isRed, 212), 219);
    public Point2d P6M = new Point2d(getXPosition(isRed, P6.x + 20), P6.y);

    public Point2d P7 = new Point2d(getXPosition(isRed, 212), 276);
    public Point2d P7M = new Point2d(getXPosition(isRed, P7.x + 20), P7.y);

    public Point2d P8 = new Point2d(getXPosition(isRed, 0), 29.64);
    public Point2d P8M = new Point2d(getXPosition(isRed, P8.x + 20), P8.y);

    public Point2d P9 = new Point2d(getXPosition(isRed, 0), 95.64);
    public Point2d P9M = new Point2d(getXPosition(isRed, P9.x + 20), P9.y);

    public Point2d P10 = new Point2d(getXPosition(isRed, 0), 161.64);
    public Point2d P10M = new Point2d(getXPosition(isRed, P10.x + 20), P10.y);
    
    public Point2d P11 = new Point2d(getXPosition(isRed, 0), 227.64);
    public Point2d P11M = new Point2d(getXPosition(isRed, P11.x + 20), P11.y);

    public Point2d P12 = new Point2d(getXPosition(isRed, 0), 293.64);
    public Point2d P12M = new Point2d(getXPosition(isRed, P12.x + 20), P12.y);

    //ToDO fix 13 14
    public Point2d P13 = new Point2d(getXPosition(isRed, 0), 93.154754 - halfFramePreremetere);
    public Point2d P14 = new Point2d(getXPosition(isRed, 0), 231.777 + halfFramePreremetere);
    //ToDo : add p15, p16, p17
    public Point2d P15 = new Point2d(0,0);//Might use

    public Point2d P16 = new Point2d(getXPosition(isRed, 172.955),  93.154754 - halfFramePreremetere);
    public Point2d P17 = new Point2d(getXPosition(isRed, 172.955),  231.777 + halfFramePreremetere);

    public Point2d P18 = new Point2d(getXPosition(isRed, 115 + 21 + halfFramePreremetere), 162);

    public Point2d P19 = new Point2d(getXPosition(isRed, 204), 107);
    public Point2d P20 = new Point2d(getXPosition(isRed, 204), 217);

    public Point2d P21 = new Point2d(getXPosition(isRed, 72), 0);

    public static double getXPosition(boolean isRed, double position)
    {
        if(isRed)
        {
            return position;
        }
        else
        {
            return position * -1.0;
        }
    }
}
