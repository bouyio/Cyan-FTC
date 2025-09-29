package com.github.bouyio.cyancore.pathing.engine;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Loggable;
import com.github.bouyio.cyancore.debugger.formating.Identifier;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.localization.PositionProvider;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Point;
import com.github.bouyio.cyancore.pathing.Path;
import com.github.bouyio.cyancore.pathing.PointSequence;
import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.MathUtil;
import com.github.bouyio.cyancore.util.PIDController;

/**
 * A system to follow {@link Point}/{@link PointSequence}/{@link Path} based on the robot's provided position.
 * Outputs the calculations to {@code differential drivetrain} motor powers.
 * Optimized for performance with cached calculations and improved algorithms.
 *
 * @author Bouyio (<a href="https://github.com/bouyio">...</a>)
 * @author Gvol (<a href="https://github.com/Gvolexe">...</a>)
 */
public class PathFollower {

    // ----SYSTEM COMPONENTS----

    PIDController controller;
    PositionProvider posProvider;
    VectorInterpreter vectorInterpreter;

    CircleLineIntersectionCalculator cliCalc;
    private Logger logger;

    // ----USER SETTINGS----

    private Distance.DistanceUnit distanceUnitOfMeasurement = null;
    private double distanceErrorTolerance = 0;

    // ---SYSTEM WORKING VARIABLES---
    private double[] motorPowers;

    private boolean isLoggerAttached = false;

    // ----SYSTEM VERSION INFO---

    private final String SYSTEM_VERSION = "2.0";
    private final String SYSTEM_NAME = "PATH_FOLLOWER";
    public String getSystemVersion() {return SYSTEM_VERSION;}
    public String getSystemName() {return SYSTEM_NAME;}

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
    public PathFollower(PositionProvider posProvider, VectorInterpreter vectorInterpreter) {
        // Using default proportional-only controller.
        this(posProvider, vectorInterpreter, new PIDController(1, 0, 0));
    }

    /**
     *<p> Creates a follower with a user specified {@link PIDController}.<p/>
     * @param posProvider The system used for robot localization.
     * @param controller The controller used for point following.
     */
    public PathFollower(PositionProvider posProvider, VectorInterpreter vectorInterpreter, PIDController controller) {
        this.posProvider = posProvider;
        this.controller = controller;
        this.vectorInterpreter = vectorInterpreter;
    }

    // ----SET UP METHODS----
    /**
     * <p>Sets up an {@link CircleLineIntersectionCalculator} instance which is required for {@link Path} following.<p/>
     * @param lookAheadDistance The radius of the circle used for the circle line intersection path following.
     * @param admissibleError The tolerance for point error.
     * */
    public void purePursuitSetUp(double lookAheadDistance, double admissibleError) {
        cliCalc = new CircleLineIntersectionCalculator(posProvider, lookAheadDistance, admissibleError);
    }

    /**
     * <p>Sets the minimum admissible error. It is used to determine sequence point switching and point arrival.<p/>
     * */
    public void setDistanceErrorTolerance(double tolerance) {
        this.distanceErrorTolerance = tolerance;
    }

    /**
     * <p>Sets the unit of measurement used by the follower. Required for conversions of {@link SmartPoint}.<p/>
     * */
    public void setDistanceUnitOfMeasurement(Distance.DistanceUnit unit) {
        distanceUnitOfMeasurement = unit;
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
    private Pose2D calculatePointError(Point point) {
        // Optimized: Add null check for safety
        if (point == null) {
            throw new IllegalArgumentException("Point cannot be null");
        }

        posProvider.update();

        // Optimized: Cache pose to avoid multiple calls
        Pose2D currentPose = posProvider.getPose();
        Pose2D pointCoords = point.getAsPose();

        double deltaX = pointCoords.getX() - currentPose.getX();
        double deltaY = pointCoords.getY() - currentPose.getY();

        // Optimized: Use Math.hypot for better numerical stability
        double distanceToPoint = Math.hypot(deltaX, deltaY);
        double angleError = MathUtil.wrapAngle(Math.atan2(deltaY, deltaX) - currentPose.getTheta());
        angleError = Math.toRadians(MathUtil.shiftAngle(Math.toDegrees(angleError), 0));

        // Store debug values
        dbgDistanceToPoint = distanceToPoint;
        dbgAngleError = angleError;

        return new Pose2D(deltaX, deltaY, angleError);
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

        Pose2D error = calculatePointError(point);

        double denominator = point.getDistanceFromOrigin();

        // Failsafe for dividing by zero.
        if (point.getDistanceFromOrigin() == 0)
            denominator = point.getDistanceFrom(posProvider.getPose());

        double x = error.getX() / denominator;
        double y = error.getY() / denominator;

        System.out.println(point.getDistanceFromOrigin());

        double steeringPIDOut = controller.update(error.getTheta());

        vectorInterpreter.process(new Pose2D(x, y, steeringPIDOut));
        motorPowers = vectorInterpreter.getMotorInputs();
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
        double error = Math.hypot(
                calculatePointError(currentPoint).getX(),
                calculatePointError(currentPoint).getY());

        if (error < distanceErrorTolerance) {
            currentPoint = seq.nextPoint();

            if (seq.getUnitOfMeasurement() != null && distanceUnitOfMeasurement != null) {
                currentPoint = convertToLocalUnit(currentPoint, seq.getUnitOfMeasurement());
            }
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
        posProvider.update();
        if (point == null || point.getDistanceFrom(posProvider.getPose()) < distanceErrorTolerance) {
            vectorInterpreter.stop();
            motorPowers = vectorInterpreter.getMotorInputs();
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
        return motorPowers;
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

        path.setMinimumPathError(distanceErrorTolerance);

        cliCalc.setTargetPath(path);

        Point targetPoint = cliCalc.getTargetPoint();

        if (path.getDistanceUnitOfMeasurement() != null && distanceUnitOfMeasurement != null) {
            targetPoint = convertToLocalUnit(targetPoint, path.getDistanceUnitOfMeasurement());
        }

        followPoint(targetPoint);
    }

    /**
     *
     * <p>
     *     Converts and sets the follow target of the robot to the given smart point.
     * </p>
     * <p>
     *     If the method cannot convert the smart point to the unit of measurement of the follower,
     *     because it has not been set, it uses the raw coordinates of the smart point.
     * </p>
     *
     * @param point The given smart point.
     * @implNote Calls {@link PositionProvider#update()}.
     * */
    public void followSmartPoint(SmartPoint point) {
        if (distanceUnitOfMeasurement != null) {
            followPoint(point.getAsPoint(distanceUnitOfMeasurement));
            return;
        }
        followPoint(point.getAsPoint());
    }

    /**
     *
     * <p>
     *     Converts the coordinates of a given point to a given distance unit.
     * </p>
     *
     * @param point The given point.
     * @param unit The given distance unit.
     * */
    private Point convertToLocalUnit(Point point, Distance.DistanceUnit unit) {
        SmartPoint conversionPoint = new SmartPoint(
                unit,
                point.getCoordinates().getCartesianX(),
                point.getCoordinates().getCartesianY());

        return conversionPoint.getAsPoint(distanceUnitOfMeasurement);
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
        if (cliCalc != null) cliCalc.attachLogger(logger);
    }

    /**
     * <p>
     *    Runs the debug actions, such as logging, of this system and the encapsulated systems within it.
     * <p/>
     * */
    public void debug() {
        if (isLoggerAttached) {
            logger.record(new DebugPacket<>(new Identifier(SYSTEM_NAME), SYSTEM_VERSION));
            logger.logValue("TargetPoint", dbgTargetPoint);
            logger.logValue("robotDistanceToPoint", dbgDistanceToPoint);
            logger.logValue("robotHeadingError", dbgAngleError);

            if (cliCalc != null) cliCalc.debug();
            if (vectorInterpreter instanceof Loggable) ((Loggable) vectorInterpreter).log();
        }
    }
}
