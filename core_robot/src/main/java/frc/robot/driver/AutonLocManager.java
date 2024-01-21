
@Singleton


public class AutonLocManager {

    private final ILogger logger;

    private final TrajectoryManager trajectoryManager;
    private final SmartDashboardSelectionManager selectionManager;
    private final IDriverStation driverStation;

    public static Point2d P1 = calculateSide(324.173750 - HardwareConstants.HALF_FRAME_PERIMETER, 253.611250 - HardwareConstants.HALF_FRAME_PERIMETER);
    public static Point2d P2 = calculateSide(299.199673 - 0 , 78.285726 - 0); //X and Y need to be figured out and stubstracted
    public static Point2d P3 = calculateSide(250.5 + HardwareConstants.HALF_FRAME_PERIMETER, 324.0 + HardwareConstants.HALF_FRAME_PERIMETER);
    public static Point2d P4 = calculateSide(288.432368 - HardwareConstants.HALF_FRAME_PERIMETER, 116.473567 + HardwareConstants.HALF_FRAME_PERIMETER);
    public static Point2d P5 = calculateSide(212.6, 46.888409);
    public static Point2d P6 = calculateSide(212.6, 103.888409);
    public static Point2d P7 = calculateSide(212.6, 160.888409);
    public static Point2d P8 = calculateSide(0, 29.64);
    public static Point2d P9 = calculateSide(0, 95.64);
    public static Point2d P10 = calculateSide(0, 161.64);
    public static Point2d P11 = calculateSide(0, 227.64);
    public static Point2d P12 = calculateSide(0, 293.64);

    @Inject

    public AutonLocManager (
        LoggingManager logger,
        TrajectoryManager trajectoryManager,
        SmartDashboardSelectionManager selectionManager,
        IRobotProvider provider
    )

    {
        //find alliance color from driverStation
        Optional<Alliance> alliance = this.driverStation.getAlliance();
            boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
            this.logger.logString("(" + (isRed ? "red" : "blue") + ")");

        public Point2d calculateSide(double valueX, double valueY) {

            //TODO: replace red and blue calculations with real calculations
            //FOR X VALUES, Blue is Positive, Red is Negative
            //placeholder changes for Red as of now

            private double newValueX;
            private double newValueY;

            if (isRed == false) {
                newValueX = valueX;
                newValueY = valueY;
            }
            else {
                newValueX = -1 * valueX;
                newValueY = valueY + 0.001;
            }

            return new Point2d(newValueX, newValueY);
        }
    }
}
