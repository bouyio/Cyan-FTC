package bouyio.cyancore;

import bouyio.cyancore.debugger.Logger;
import bouyio.cyancore.geomery.Pose2D;
import bouyio.cyancore.util.MathUtil;

/**
 * <p>
 *     Utilizes the gyroscope and the drive encoders of a differential driving base to determine its position.
 *     In order to calculate it, trigonometric functions are used.
 * <p/>
 * <p>
 *     Note: The distance measuring units are entirely determined by the input from the encoders.
 *     To avoid error and inconsistencies, be sure to convert the input to same units used for pathing.
 * <p/>
 * @see bouyio.cyancore.PositionProvider
 * @see Pose2D
 * */
public class GyroTankOdometry implements PositionProvider {

    private Pose2D currPose;

    private double x;
    private double y;
    private double theta;

    private final double thetaOffset;

    private double previousLeft = 0;
    private double previousRight = 0;

    private double currentLeft = 0;
    private double currentRight = 0;

    private double previousAngle = 0;
    private double deltaAngle = 0;

    private Logger logger;
    private boolean isLoggerAttached = false;

    /**<p>Creates a position tracker at the default starting position; (0,0).<p/>*/
    public GyroTankOdometry() {
        this(0, 0, 0);
    }

    /**
     * <p>Creates a position tracker at a specified position.<p/>
     * @param initialX The initial x coordinates of the robot.
     * @param initialY The initial y coordinates of the robot.
     * @param initialHeading The initial heading of the robot in Degrees.
     * */
    public GyroTankOdometry(double initialX, double initialY, double initialHeading) {
        x = initialX;
        y = initialY;
        thetaOffset = initialHeading;
    }

    /**
     * <p>Formats the x, y and heading of the robot as {@link Pose2D}.<p/>
     * @return The x, y, and heading in their respective units.
     * */
    @Override
    public Pose2D getPose() {
        return currPose;
    }

    /**
     * <p>
     *     Refreshes the measurements from the drive encoders and the gyroscope.
     *     Use before calling {@link #update()}.
     * <p/>
     * @param left The input from the left drive encoder.
     * @param right The input from the right drive encoder.
     * @param angle The input from the gyroscope.
     * */
    public void updateMeasurements(double left, double right, double angle) {
        currentLeft = left;
        currentRight = right;

        theta = Math.toRadians(MathUtil.shiftAngle(angle, thetaOffset));
    }

    /**
     * <p>Updates the position estimate.<p/>
     * <p>Note: before calling be sure to update the measurements with {@link #updateMeasurements(double, double, double)}.<p/>
     * */
    @Override
    public void update() {
        double dLeft = currentLeft - previousLeft;
        double dRight = currentRight - previousRight;

        double dC = (dLeft + dRight) / 2;

        double dX = dC * Math.cos(theta);
        double dY = dC * Math.sin(theta);

        previousLeft = currentLeft;
        previousRight = currentRight;

        x += dX;
        y += dY;
        currPose = new Pose2D(x, y, theta);
    }

    /**
     * <p>
     *    Attaches a logger to this instance to record debug values.
     * <p/>
     * */
    public void attachLogger(Logger logger) {
        this.logger = logger;
        isLoggerAttached = true;
    }

    /**
     * <p>
     *    Runs the debug actions, such as logging, of this system and the encapsulated systems within it.
     * <p/>
     * */
    public void debug() {
        if (isLoggerAttached) {
            logger.logValue("robotX", x);
            logger.logValue("robotY", y);
            logger.logValue("robotHeading", theta);
        }
    }
}
