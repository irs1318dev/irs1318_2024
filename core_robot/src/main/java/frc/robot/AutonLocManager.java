package frc.robot;
import java.util.Optional;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IDriverStation;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.Point2d;

@Singleton
public class AutonLocManager
{

    private boolean isRed;
    private IDriverStation driverStation;

    public Point2d P1;
    public Point2d P2;
    public Point2d P3;
    public Point2d P4;
    public Point2d P5;
    public Point2d P5M;
    public Point2d P6;
    public Point2d P6M;
    public Point2d P7;
    public Point2d P7M;
    public Point2d P8;
    public Point2d P8M;
    public Point2d P9;
    public Point2d P9M;
    public Point2d P10;
    public Point2d P10M;
    public Point2d P11;
    public Point2d P11M;
    public Point2d P12;
    public Point2d P12M;
    public Point2d P13;
    public Point2d P14;
    public Point2d P15;
    public Point2d P16;
    public Point2d P17;
    public Point2d P18;
    public Point2d P19;
    public Point2d P20;
    public Point2d P21;

    public AutonLocManager(boolean isRed)
    {
        this.isRed = isRed;
        this.setValues();
    }

    @Inject
    public AutonLocManager(IRobotProvider provider) 
    {
        this.driverStation = provider.getDriverStation();
    }

    public void updateAlliance()
    {
        Optional<Alliance> alliance = driverStation.getAlliance();
        this.isRed = alliance.isPresent() || alliance.get() == Alliance.Red;
        this.setValues();
    }

    public double getOrientationOrHeading(double orientationOrHeading)
    {
        return AutonLocManager.getOrientationOrHeading(this.isRed, orientationOrHeading);
    }

    public boolean getIsRed()
    {
        return this.isRed;
    }

    private void setValues()
    {
        //Red is positive
        //Blue is negative

        //Y field length: 323in
        //X field length: 653in

        this.P1 = new Point2d(AutonLocManager.getXPosition(this.isRed, 326.5 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER), 64 + HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P2 = new Point2d(AutonLocManager.getXPosition(isRed, 289 - 21.22) , 198); //210 degrees orientation
        this.P3 = new Point2d(AutonLocManager.getXPosition(isRed, 250.5 + HardwareConstants.ROBOT_HALF_FRAME_PERIMETER), 306 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P4 = new Point2d(AutonLocManager.getXPosition(isRed, 288 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER), 239 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P5 = new Point2d(AutonLocManager.getXPosition(isRed, 204), 162);//212), 162);
        this.P5M = new Point2d(AutonLocManager.getXPosition(isRed, P5.x + 20), P5.y);
        this.P6 = new Point2d(AutonLocManager.getXPosition(isRed, 212), 219);
        this.P6M = new Point2d(AutonLocManager.getXPosition(isRed, P6.x + 20), P6.y);
        this.P7 = new Point2d(AutonLocManager.getXPosition(isRed, 212), 276);
        this.P7M = new Point2d(AutonLocManager.getXPosition(isRed, P7.x + 20), P7.y);
        this.P8 = new Point2d(AutonLocManager.getXPosition(isRed, 0), 29.64);
        this.P8M = new Point2d(AutonLocManager.getXPosition(isRed, P8.x + 20), P8.y);
        this.P9 = new Point2d(AutonLocManager.getXPosition(isRed, 0), 95.64);
        this.P9M = new Point2d(AutonLocManager.getXPosition(isRed, P9.x + 20), P9.y);
        this.P10 = new Point2d(AutonLocManager.getXPosition(isRed, 0), 161.64);
        this.P10M = new Point2d(AutonLocManager.getXPosition(isRed, P10.x + 20), P10.y);
        this.P11 = new Point2d(AutonLocManager.getXPosition(isRed, 0), 227.64);
        this.P11M = new Point2d(AutonLocManager.getXPosition(isRed, P11.x + 20), P11.y);
        this.P12 = new Point2d(AutonLocManager.getXPosition(isRed, 0), 293.64);
        this.P12M = new Point2d(AutonLocManager.getXPosition(isRed, P12.x + 20), P12.y);

        //ToDO fix 13 14
        this.P13 = new Point2d(AutonLocManager.getXPosition(isRed, 0), 93.154754 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P14 = new Point2d(AutonLocManager.getXPosition(isRed, 0), 231.777 + HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);

        //ToDo : add p15, p16, p17
        this.P15 = new Point2d(0,0); //Might use
        this.P16 = new Point2d(AutonLocManager.getXPosition(isRed, 172.955),  93.154754 - HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P17 = new Point2d(AutonLocManager.getXPosition(isRed, 172.955),  231.777 + HardwareConstants.ROBOT_HALF_FRAME_PERIMETER);
        this.P18 = new Point2d(AutonLocManager.getXPosition(isRed, 115 + 21 + HardwareConstants.ROBOT_HALF_FRAME_PERIMETER), 162);
        this.P19 = new Point2d(AutonLocManager.getXPosition(isRed, 204), 107);
        this.P20 = new Point2d(AutonLocManager.getXPosition(isRed, 204), 217);
        this.P21 = new Point2d(AutonLocManager.getXPosition(isRed, 72), 0);
    }

    private static double getOrientationOrHeading(boolean isRed, double orientationOrHeading)
    {
        if (isRed)
        {
            return orientationOrHeading;
        }
        else
        {
            return 180.0 - orientationOrHeading;
        }
    }

    private static double getXPosition(boolean isRed, double position)
    {
        if (isRed)
        {
            return position;
        }
        else
        {
            return position * -1.0;
        }
    }
}
