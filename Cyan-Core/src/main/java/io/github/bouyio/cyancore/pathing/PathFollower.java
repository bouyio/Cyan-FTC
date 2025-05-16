package io.github.bouyio.cyancore.pathing;

import io.github.bouyio.cyancore.PositionProvider;
import io.github.bouyio.cyancore.debugger.Logger;
import io.github.bouyio.cyancore.geomery.Point;
import io.github.bouyio.cyancore.util.MathUtil;
import io.github.bouyio.cyancore.util.PIDController;

/**
 * <p>
 *     A system to follow {@link Point}/{@link PointSequence}/{@link Path} based on the robot's provided position.
 *     Outputs the calculations to {@code differential drivetrain} motor powers.
 * <p/>
 * */
public class PathFollower {

    PIDController controller;
    PositionProvider posProvider;

    IntersectionTargetCalculator purePursuit;

    IntersectionV2 cliCalc;

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

    /**
     *<p> Creates a follower with the default proportional-only controller.<p/>
     * @param posProvider The system used for robot localization.
     * */
    public PathFollower(PositionProvider posProvider) {
        // Using default proportional-only controller.
        this(posProvider, new PIDController(1, 0, 0));
    }

    /**
     *<p> Creates a follower with a user specified {@link PIDController}.<p/>
     * @param posProvider The system used for robot localization.
     * @param controller The controller used for point following.
     */
    public PathFollower(PositionProvider posProvider, PIDController controller) {
        this.posProvider = posProvider;
        this.controller = controller;
    }

    // ----SET UP METHODS----
    /**
     * <p>Sets up an {@link IntersectionV2} instance which is required for {@link Path} following.<p/>
     * @param lookAheadDistance The radius of the circle used for the circle line intersection path following.
     * @param admissibleError The tolerance for point error.
     * */
    public void purePursuitSetUp(double lookAheadDistance, double admissibleError) {
        cliCalc = new IntersectionV2(posProvider, lookAheadDistance, admissibleError);
    }

    /**
     * <p>Sets the minimum admissible error. It is used to determine sequence point switching and point arrival.<p/>
     * */

    public void setDistanceErrorTolerance(double tolerance) {
        this.distanceErrorTolerance = tolerance;
    }

    // ----POINT/SEQUENCE/PATH FOLLOWING----

    /**
     *
     * <p>
     *     Calculates the linear and angular error of the provided point relative to the robots position and heading.
     *     Returns it in the error project standard. {@code linear}-{@code angular}
     * <p/>
     *
     * @param point The point that its error is to be calculated.
     * @return Linear [undefined] and angular [Radians] error relative to the robot.
     * @implNote Calls {@link PositionProvider#update()}.
     * */
    private double[] calculatePointError(Point point) {
        posProvider.update();

        double deltaX = point.getCoordinates().getX() - posProvider.getPose().getX();
        double deltaY = point.getCoordinates().getY() - posProvider.getPose().getY();

        double distanceToPoint = MathUtil.hypotenuse(deltaX, deltaY);
        double angleError = MathUtil.wrapAngle(Math.atan2(deltaY, deltaX) - posProvider.getPose().getTheta());

        dbgDistanceToPoint = distanceToPoint;
        dbgAngleError = angleError;

        return new double[] {distanceToPoint, angleError};
    }

    /**
     *
     * <p>
     *     Calculates the drivetrain motor powers by normalizing the error and stores them internally for retrieval.
     *     To use the calculated result please call {@link #getCalculatedPowers()}.
     * <p/>
     *
     * @param point The target point.
     * @implNote Calls {@link PositionProvider#update()}.
     * */
    private void calculatePowers(Point point) {

        double[] error = calculatePointError(point);

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
     * <p>
     *      Follows each every point of a {@link PointSequence} one by one.
     *      Uses the distance tolerance set in {@link #setDistanceErrorTolerance(double)} to trigger
     *      a switch to the next point. Uses the point following algorithm found in {@link #followPoint(Point)}.
     * <p/>
     *
     * <p>To get the output powers of the calculation, please use {@link #getCalculatedPowers()}.<p/>
     *
     * @param seq The sequence to be followed.
     * @return Returns if the follower can follow the sequence; it isn't finished or is null.
     * @implNote Calls {@link PositionProvider#update()}.
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

    /**
     *
     * <p>
     *      Follows a specified {@code point} by calculating its relative angle to the robot and its distance from it.
     *      If the point is null the method returns.
     * <p/>
     *
     * <p>To get the output powers of the calculation, please use {@link #getCalculatedPowers()}.<p/>
     *
     * @param point The point to be followed.
     * @implNote Calls {@link PositionProvider#update()}.
     *
     * */
    public void followPoint(Point point) {
        if (point == null || point.getDistanceFrom(posProvider.getPose()) < distanceErrorTolerance) {
            steeringPower = 0;
            linearPower = 0;
            return;
        }
        dbgTargetPoint = point.toString();

        calculatePowers(point);
    }

    /**
     * <p>The calculated motor powers are returned in the standards of this project. {@code left} - {@code right}<p/>
     * @return The calculated power of each motor for Point/Path/Sequence following.
     * */
    public double[] getCalculatedPowers() {
        return new double[]{linearPower, steeringPower};
    }

    /**
     *
     * <p>
     *   Uses the circle line intersection pure pursuit algorithm to calculate the optimal point of path to be followed.
     *   Then follows the target point.
     * <p/>
     *
     * @param path The path to be followed.
     */
    public void followPath(Path path) {
        if (cliCalc == null || path == null) return;

        cliCalc.setTargetPath(path);

        Point targetPoint = cliCalc.getTargetPoint();

        followPoint(targetPoint);
    }


    // ----DEBUG METHODS----

    /**
     * <p>
     *    Attaches a logger to this instance of the follower and all the encapsulated systems within
     *    it to record debug values.
     * <p/>
     *
     * */
    public void attachLogger(Logger logger) {
        this.logger = logger;
        isLoggerAttached = true;
        if (purePursuit != null) purePursuit.attachLogger(logger);
        if (cliCalc != null) cliCalc.attachLogger(logger);
    }

    /**
     * <p>
     *    Runs the debug actions, such as logging, of this system and the encapsulated systems within it.
     * <p/>
     * */
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
    public void legacyPurePursuitSetUp(double lookAheadDistance) {
        purePursuit = new IntersectionTargetCalculator(lookAheadDistance, posProvider);
    }
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
