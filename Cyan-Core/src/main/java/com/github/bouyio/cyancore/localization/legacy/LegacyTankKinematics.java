package com.github.bouyio.cyancore.localization.legacy;

import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.localization.PositionProvider;
import com.github.bouyio.cyancore.util.MathUtil;

/**
 * <p>
 *     Utilizes the drive encoders of a differential driving base to determine its position.
 *     In order to calculate it, trigonometric functions are used.
 *     For the calculation of the heading it uses differential equations.
 * <p/>
 * <p>
 *     Note: The distance measuring units are entirely determined by the input from the encoders.
 *     To avoid error and inconsistencies, be sure to convert the input to same units used for pathing.
 * <p/>
 * <p>
 *     Note: Although it is not supported and its use should be avoided, this class is still kept for compatibility reasons.
 * <p/>
 * @see PositionProvider
 * @see Pose2D
 * */
public class LegacyTankKinematics implements PositionProvider {
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

    /**
     * <p>Creates a position tracker at a specified position.<p/>
     * @param initialX The initial x coordinates of the robot.
     * @param initialY The initial y coordinates of the robot.
     * @param initialHeading The initial heading of the robot in Degrees.
     * @param trackWidth The distance between the centers of the two wheels - used for heading calculation.
     * */
    public LegacyTankKinematics(double initialX, double initialY, double initialHeading, double trackWidth) {
        x = initialX;
        y = initialY;
        theta = initialHeading;

        TRACK_WIDTH = trackWidth;
    }

    /**
     * <p>Creates a position tracker at the default starting position; (0,0).<p/>
     * @param trackWidth The distance between the centers of the two wheels - used for heading calculation.
     * */
    public LegacyTankKinematics(double trackWidth) {
        this(0, 0, 0, trackWidth);
    }

    /**
     * <p>Formats the x, y and heading of the robot as {@link Pose2D}.<p/>
     * @return The x, y, and heading in their respective units.
     * */
    @Override
    public Pose2D getPose() {
        return currentPose;
    }

    /**
     * <p>
     *     Refreshes the measurements from the drive encoders.
     *     Use before calling {@link #update()}.
     * <p/>
     * @param left The input from the left drive encoder.
     * @param right The input from the right drive encoder.
     * */
    public void updateMeasurements(double left, double right) {
        currentLeft = left;
        currentRight = right;

        theta = Math.toRadians(MathUtil.shiftAngle(Math.toDegrees(theta), 0));
    }

    /**
     * <p>Updates the position and heading estimate.<p/>
     * <p>
     *     Heading is calculated using {@code θ' = θ + (ΔR - ΔL) / d},
     *     where {@code θ} the previous heading, where {@code θ'} the current heading,
     *     where {@code ΔR} the displacement of the right wheel,
     *     where {@code ΔL} the displacement of the left wheel,
     *     where {@code d} the trackwidth (the distance between the centers of the two wheels)
     * <p/>
     * <p>Note: before calling be sure to update the measurements with {@link #updateMeasurements(double, double)}.<p/>
     * */
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
     *    Runs the debug actions, such as logging, of this system.
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
