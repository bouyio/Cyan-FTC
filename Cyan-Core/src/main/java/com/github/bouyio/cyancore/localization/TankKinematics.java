package com.github.bouyio.cyancore.localization;

import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartVector;
import com.github.bouyio.cyancore.util.Distance;

import java.util.function.DoubleSupplier;

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
 * @see PositionProvider
 * @see Pose2D
 * */
public class TankKinematics implements PositionProvider {

    public static class TankKinematicsMeasurementProvider {
        public final DoubleSupplier leftEncoderValueProvider;
        public final DoubleSupplier rightEncoderValueProvider;
        private final double ticksToDistance;

        public TankKinematicsMeasurementProvider(
                DoubleSupplier leftEncoderValueProvider,
                DoubleSupplier rightEncoderValueProvider,
                double ticksToDistance
        ) {
            this.leftEncoderValueProvider = leftEncoderValueProvider;
            this.rightEncoderValueProvider = rightEncoderValueProvider;
            this.ticksToDistance = ticksToDistance;
        }

        public double getLeftWheelDistance() {
            return leftEncoderValueProvider.getAsDouble() * ticksToDistance;
        }

        public double getRightWheelDistance() {
            return rightEncoderValueProvider.getAsDouble() * ticksToDistance;
        }
    }

    private final TankKinematicsMeasurementProvider measurementProvider;
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
     * @param initialHeading The initial heading of the robot in Degrees.
     * @param trackWidth The distance between the centers of the two wheels - used for heading calculation.
     * */
    public TankKinematics(SmartVector initialPosition, double initialHeading, double trackWidth, TankKinematicsMeasurementProvider measurementProvider) {
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
     * */
    public TankKinematics(double trackWidth, Distance.DistanceUnit distanceUnitOfMeasurement, TankKinematicsMeasurementProvider measurementProvider) {
        this(new SmartVector(distanceUnitOfMeasurement, 0, 0), 0, trackWidth, measurementProvider);
    }

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
     * <p>Updates the position and heading estimate.<p/>
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
