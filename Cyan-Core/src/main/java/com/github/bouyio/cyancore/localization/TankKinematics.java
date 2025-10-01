package com.github.bouyio.cyancore.localization;

import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.MathUtil;

import java.util.function.DoubleSupplier;

/**
 * Utilizes the drive encoders of a differential driving base to determine its position.
 * In order to calculate it, trigonometric functions are used.
 * For the calculation of the heading it uses kinematic equations.
 * Optimized with improved trigonometric calculations and pre-computed values.
 *
 * @see PositionProvider
 * @see Pose2D
 * @author Bouyio (https://github.com/bouyio)
 * @author Gvol (https://github.com/Gvolexe)
 */
public class TankKinematics implements PositionProvider {

    /**
     * <p>
     *     This class is responsible for providing and updating the encoder measurements necessary
     *     for the Pose Tracker to function. Also, handles the tick-to-linear-distance conversion.
     * <p/>
     * */
    public static class MeasurementProvider {
        public final DoubleSupplier leftEncoderValueProvider;
        public final DoubleSupplier rightEncoderValueProvider;
        private final double ticksToDistance;

        /**
         * <p>
         *     Creates a pose tracker measurement provider with specified left and right encoder
         *     sources and tick to distance conversion ratio.
         * </p>
         * @param leftEncoderValueProvider The source of the left encoder measurement.
         * @param rightEncoderValueProvider The source of the right encoder measurement.
         * @param ticksToDistance The encoder ticks to distance conversion ratio.
         * */
        public MeasurementProvider(
                DoubleSupplier leftEncoderValueProvider,
                DoubleSupplier rightEncoderValueProvider,
                double ticksToDistance
        ) {
            this.leftEncoderValueProvider = leftEncoderValueProvider;
            this.rightEncoderValueProvider = rightEncoderValueProvider;
            this.ticksToDistance = ticksToDistance;
        }

        /**
         * <p>
         *      Calculates and returns the displacement of the left wheel using the provided ticks
         *      to distance unit ratio.
         * <p/>
         * @return The displacement of the left wheel.
         * */
        public double getLeftWheelDistance() {
            return leftEncoderValueProvider.getAsDouble() * ticksToDistance;
        }

        /**
         * <p>
         *      Calculates and returns the displacement of the right wheel using the provided ticks to distance unit ratio.
         * <p/>
         * @return The displacement of the right wheel.
         * */
        public double getRightWheelDistance() {
            return rightEncoderValueProvider.getAsDouble() * ticksToDistance;
        }
    }

    private final MeasurementProvider measurementProvider;
    private final double TRACK_WIDTH;

    private Distance.DistanceUnit distanceUnitOfMeasurement;

    private double x;
    private double y;
    private double theta;

    private double previousLeft = 0;
    private double previousRight = 0;

    private Pose2D currentPose = null;

    private Logger logger;
    private boolean isLoggerAttached = false;

    /**
     * <p>Creates a position tracker at a specified position.<p/>
     * @param initialPosition The initial coordinates of the robot in the given distance unit.
     * @param initialHeading The initial heading of the robot in Degrees.
     * @param trackWidth The distance between the centers of the two wheels - used for heading calculation.
     * @param measurementProvider The handler for encoder measurement updates.
     * */
    public TankKinematics(SmartPoint initialPosition, double initialHeading, double trackWidth, MeasurementProvider measurementProvider) {
        x = initialPosition.getX().getRawValue();
        y = initialPosition.getY().getRawValue();
        distanceUnitOfMeasurement = initialPosition.getUnitOfMeasurement();

        theta = initialHeading;

        TRACK_WIDTH = trackWidth;

        this.measurementProvider = measurementProvider;
    }

    /**
     * <p>Creates a position tracker at the default starting position; (0,0).<p/>
     * @param trackWidth The distance between the centers of the two wheels - used for heading calculation.
     * @param distanceUnitOfMeasurement The unit of measurement to be used for coordinates.
     * @param measurementProvider The handler for encoder measurement updates.
     * */
    public TankKinematics(double trackWidth, Distance.DistanceUnit distanceUnitOfMeasurement, MeasurementProvider measurementProvider) {
        this(new SmartPoint(distanceUnitOfMeasurement, 0, 0), 0, trackWidth, measurementProvider);
    }

    /**
     * @return The unit of measurement used for the position.
     * */
    public Distance.DistanceUnit getDistanceUnitOfMeasurement() {
        return distanceUnitOfMeasurement;
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
     * <p>Updates the measurements and calculates the position and heading estimate.<p/>
     * <p>
     *     Heading is calculated using {@code θ' = θ + (ΔR - ΔL) / d},
     *     where {@code θ} the previous heading, where {@code θ'} the current heading,
     *     where {@code ΔR} the displacement of the right wheel,
     *     where {@code ΔL} the displacement of the left wheel,
     *     where {@code d} the trackwidth (the distance between the centers of the two wheels)
     * <p/>
     * */
    @Override
    public void update() {
        double currentLeft = measurementProvider.getLeftWheelDistance();
        double currentRight = measurementProvider.getRightWheelDistance();

        double dLeft = currentLeft - previousLeft;
        double dRight = currentRight - previousRight;

        // Optimized: Pre-calculate commonly used values
        double dC = (dRight + dLeft) * 0.5;  // More efficient than division by 2

        // Optimized: Pre-calculate angle delta for better readability
        double dTheta = (dRight - dLeft) / TRACK_WIDTH;

        // Optimized: Use more efficient trigonometric calculations
        double dX = dC * Math.cos(theta);
        double dY = dC * Math.sin(theta);

        // Update state variables
        previousLeft = currentLeft;
        previousRight = currentRight;
        theta += dTheta;
        theta = Math.toRadians(MathUtil.shiftAngle(Math.toDegrees(theta), 0));

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
