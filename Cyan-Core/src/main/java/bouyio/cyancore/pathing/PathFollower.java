package bouyio.cyancore.pathing;

import bouyio.cyancore.PositionProvider;
import bouyio.cyancore.debugger.Logger;
import bouyio.cyancore.geomery.Point;
import bouyio.cyancore.util.PIDController;

public class PathFollower {

    PIDController controller;

    PositionProvider posProvider;

    private double angleErrorTolerance = 0;
    private double distanceErrorTolerance = 0;

    private double steeringPower = 0;
    private double linearPower = 0;

    private Logger logger;
    private boolean isLoggerAttached = false;

    // ----DEBUGGER FIELDS----
    // The following fields are meant to be used for debug purposes ONLY.
    // NOT for bypassing variable scope.

    private double dbgDistanceToPoint = 0;
    private double dbgAngleError = 0;
    private String dbgTargetPoint = "";

    public PathFollower(PositionProvider posProvider) {
        // Using default proportional-only controller.
        this(posProvider, new PIDController(1, 0, 0));
    }

    public PathFollower(PositionProvider posProvider, PIDController controller) {
        this.posProvider = posProvider;
        this.controller = controller;
    }

    public void setAngleErrorTolerance(double tolerance) {
        this.angleErrorTolerance = tolerance;
    }

    public void setDistanceErrorTolerance(double tolerance) {
        this.distanceErrorTolerance = tolerance;
    }


    @Deprecated
    public double[] calculatePowersToPoint(Point point) {
        posProvider.update();

        double deltaX = point.getCoordinates().getX() - posProvider.getPose().getX();
        double deltaY = point.getCoordinates().getY() - posProvider.getPose().getY();

        double distanceToPoint = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        double linearPower = distanceToPoint / (Math.abs(deltaX) + Math.abs(deltaY));

        double angleError = Math.atan2(deltaY, deltaX) - posProvider.getPose().getTheta();
        double steeringPower = angleError / Math.PI;

        return new double[] {linearPower, steeringPower};
    }

    private double[] calculatePointError(Point point) {
        posProvider.update();

        double deltaX = point.getCoordinates().getX() - posProvider.getPose().getX();
        double deltaY = point.getCoordinates().getY() - posProvider.getPose().getY();

        double distanceToPoint = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        double angleError = Math.atan2(deltaY, deltaX) - posProvider.getPose().getTheta();

        dbgDistanceToPoint = distanceToPoint;
        dbgAngleError = angleError;

        return new double[] {distanceToPoint, angleError};
    }

    private void calculatePowers(Point point, double[] error) {

        // Not proud of this. We might need restructuring to avoid such duplication.
        double linearPower = error[0] /
                (Math.abs(point.getCoordinates().getX() - posProvider.getPose().getX()) +
                        Math.abs(point.getCoordinates().getY() - posProvider.getPose().getY()));

        double steeringPIDOut = controller.update(error[1]);

        double steeringPower = steeringPIDOut / Math.PI;

        this.linearPower = linearPower;
        this.steeringPower = steeringPower;
    }

    public void followPointSequence(PointSequence seq) {

        Point currentPoint = seq.getCurrentPoint();
        double[] error = calculatePointError(currentPoint);

        if (error[0] < distanceErrorTolerance && error[1] < angleErrorTolerance) {
            currentPoint = seq.nextPoint();
        }

        if (currentPoint == null) return;

        followPoint(currentPoint);

        dbgTargetPoint = currentPoint.toString();
    }

    public void followPoint(Point point) {
        calculatePowers(point, calculatePointError(point));
    }

    public double[] getCalculatedPowers() {
        return new double[]{linearPower, steeringPower};
    }

    public void attachLogger(Logger logger) {
        this.logger = logger;
        isLoggerAttached = true;
    }

    public void debug() {
        if (isLoggerAttached) {
            logger.logValue("TargetPoint", dbgTargetPoint);
            logger.logValue("linearPower", linearPower);
            logger.logValue("steeringPower", steeringPower);
            logger.logValue("robotDistanceToPoint", dbgDistanceToPoint);
            logger.logValue("robotHeadingError", dbgAngleError);
        }
    }
}
