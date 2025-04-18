package bouyio.cyancore.pathing;

import bouyio.cyancore.PositionProvider;
import bouyio.cyancore.debugger.Logger;
import bouyio.cyancore.geomery.Point;
import bouyio.cyancore.util.MathUtil;
import bouyio.cyancore.util.PIDController;
import bouyio.cyancore.util.StuckOscillationController;

public class PathFollower {

    PIDController controller;
    StuckOscillationController robotUnstucker;

    PositionProvider posProvider;

    IntersectionTargetCalculator purePursuit;

    IntersectionV2 cliCalc;

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

    // ----CONSTRUCTORS----

    public PathFollower(PositionProvider posProvider) {
        // Using default proportional-only controller.
        this(posProvider, new PIDController(1, 0, 0));
    }

    public PathFollower(PositionProvider posProvider, PIDController controller) {
        this.posProvider = posProvider;
        this.controller = controller;
        robotUnstucker = new StuckOscillationController();
    }

    // ----SET UP METHODS----
    public void purePursuitSetUp(double lookAheadDistance) {
        cliCalc = new IntersectionV2(posProvider, lookAheadDistance);
    }

    public void legacyPurePursuitSetUp(double lookAheadDistance) {
        purePursuit = new IntersectionTargetCalculator(lookAheadDistance, posProvider);
    }

    public void setAngleErrorTolerance(double tolerance) {
        this.angleErrorTolerance = tolerance;
    }

    public void setDistanceErrorTolerance(double tolerance) {
        this.distanceErrorTolerance = tolerance;
    }

    // ----POINT/SEQUENCE/PATH FOLLOWING----

    private double[] calculatePointError(Point point) {
        posProvider.update();

        double deltaX = point.getCoordinates().getX() - posProvider.getPose().getX();
        double deltaY = point.getCoordinates().getY() - posProvider.getPose().getY();

        double distanceToPoint = MathUtil.hypotenuse(deltaX, deltaY);
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

    /**
     * @return Returns if the follower can follow the sequence; it isn't finished or is null.
     * */
    public boolean followPointSequence(PointSequence seq) {

        if (seq == null) return false;

        Point currentPoint = seq.getCurrentPoint();
        double[] error = calculatePointError(currentPoint);

        if (error[0] < distanceErrorTolerance) {
            currentPoint = seq.nextPoint();
        }

        if (currentPoint == null) return false;

        followPoint(currentPoint);

        return true;
    }

    public void followPoint(Point point) {
        if (point == null) return;
        if (point.getDistanceFrom(posProvider.getPose()) < distanceErrorTolerance) {
            steeringPower = 0;
            linearPower = 0;
            return;
        }
        dbgTargetPoint = point.toString();

        calculatePowers(point, calculatePointError(point));
    }

    public double[] getCalculatedPowers() {
        return new double[]{linearPower, steeringPower};
    }

    public void followPath(Path path) {
        if (cliCalc == null || path == null) return;

        cliCalc.setTargetPath(path);

        Point targetPoint = cliCalc.getTargetPoint();

        if (targetPoint == null) {
            linearPower = 0;
            steeringPower = 0;
            return;
        }

        followPoint(targetPoint);
    }


    // ----DEBUG METHODS----

    public void attachLogger(Logger logger) {
        this.logger = logger;
        isLoggerAttached = true;
        if (purePursuit != null) purePursuit.attachLogger(logger);
        if (cliCalc != null) cliCalc.attachLogger(logger);
    }

    public void debug() {
        if (isLoggerAttached) {
            logger.logValue("TargetPoint", dbgTargetPoint);
            logger.logValue("linearPower", linearPower);
            logger.logValue("steeringPower", steeringPower);
            logger.logValue("robotDistanceToPoint", dbgDistanceToPoint);
            logger.logValue("robotHeadingError", dbgAngleError);

            if (purePursuit != null) purePursuit.debug();
            if (cliCalc != null) cliCalc.debug();
        }
    }

    // ----DO NOT USE----

    @Deprecated
    public void followPath(LegacyPath path) {
        if (path.getMinimumPointIndex() >= 0 &&
                path.getPathPoints()
                        .get(path.getMinimumPointIndex())
                        .getDistanceFrom(posProvider.getPose()) < distanceErrorTolerance) {
            path.nextPoint();
        }
        purePursuit.setTarget(path);

        purePursuit.followPath();

        followPoint(purePursuit.getGoal());
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
}
