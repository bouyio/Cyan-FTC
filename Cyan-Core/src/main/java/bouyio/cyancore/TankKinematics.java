package bouyio.cyancore;

import bouyio.cyancore.debugger.Logger;
import bouyio.cyancore.geomery.Pose2D;

public class TankKinematics implements PositionProvider {
    private final double TRACK_WIDTH;

    private double x;
    private double y;
    private double theta;

    private double previousLeft = 0;
    private double previousRight = 0;

    private double currentLeft = 0;
    private double currentRight = 0;

    private Pose2D currentPose = null;

    private Logger logger;
    private boolean isLoggerAttached = false;


    public TankKinematics(double initialX, double initialY, double initialTheta, double trackWidth) {
        x = initialX;
        y = initialY;
        theta = initialTheta;

        TRACK_WIDTH = trackWidth;
    }

    public TankKinematics(double trackWidth) {
        this(0, 0, 0, trackWidth);
    }

    @Override
    public Pose2D getPose() {
        return currentPose;
    }

    public void updateMeasurements(double left, double right) {
        currentLeft = left;
        currentRight = right;
    }

    @Override
    public void update() {
        double dLeft = currentLeft - previousLeft;
        double dRight = currentRight - previousRight;

        double dC = (dRight + dLeft) / 2;

        double dX = dC * Math.cos(theta);
        double dY = dC * Math.sin(theta);

        double heading = theta + (dRight - dLeft) / TRACK_WIDTH;

        previousLeft = currentLeft;
        previousRight = currentRight;
        theta = heading;

        x += dX;
        y += dY;
        currentPose = new Pose2D(x, y, theta);

        if (isLoggerAttached) {
            logger.logValue("robotX", x);
            logger.logValue("robotY", y);
            logger.logValue("robotHeading", theta);
        }
    }

    public void attachLogger(Logger logger) {
        this.logger = logger;
        isLoggerAttached = true;
    }
}
