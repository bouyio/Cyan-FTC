package com.github.bouyio.cyancore.localization;

import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.MathUtil;

import java.util.function.DoubleSupplier;

/**
 * Utilizes three dead-wheel encoders to estimate its position.
 * In order to calculate it, trigonometric functions are used.
 * For the calculation of the heading it uses kinematic equations.
 *
 * @see PositionProvider
 * @see Pose2D
 * @author Bouyio (<a href="https://github.com/bouyio">...</a>)
 */
public class MecanumKinematics implements PositionProvider {

    /**
     * <p>
     *     This class is responsible for providing and updating the encoder measurements necessary
     *     for the Pose Tracker to function. Also, handles the tick-to-linear-distance conversion.
     * <p/>
     * */
    public static class MeasurementProvider {
        public final DoubleSupplier leftFrontEncoderValueProvider;
        public final DoubleSupplier rightFrontEncoderValueProvider;
        public final DoubleSupplier leftBackEncoderValueProvider;
        public final DoubleSupplier rightBackEncoderValueProvider;
        private final double TICK_TO_DISTANCE;

        /**
         * <p>
         *     Creates a pose tracker measurement provider with specified dead-wheel encoder
         *     sources and tick to distance conversion ratio.
         * </p>
         * @param rightFrontEncoderValueProvider The source of the left parallel encoder measurement.
         * @param leftBackEncoderValueProvider The source of the right parallel encoder measurement.
         * @param leftFrontEncoderValueProvider The source of the perpendicular encoder measurement.
         * @param ticksToDistance The encoder ticks to distance conversion ratio.
         * */
        public MeasurementProvider(
                DoubleSupplier leftFrontEncoderValueProvider,
                DoubleSupplier rightFrontEncoderValueProvider,
                DoubleSupplier leftBackEncoderValueProvider,
                DoubleSupplier rightBackEncoderValueProvider,
                double ticksToDistance
        ) {
            this.leftFrontEncoderValueProvider = leftFrontEncoderValueProvider;
            this.rightFrontEncoderValueProvider = rightFrontEncoderValueProvider;
            this.leftBackEncoderValueProvider = leftBackEncoderValueProvider;
            this.rightBackEncoderValueProvider = rightBackEncoderValueProvider;
            TICK_TO_DISTANCE = ticksToDistance;
        }

        /**
         * <p>
         *      Calculates and returns the displacement of the perpendicular dead wheel using the provided ticks to distance unit ratio.
         * <p/>
         * @return The displacement of the perpendicular dead wheel.
         * */
        public double getLeftFrontWheelDistance() {
            return leftFrontEncoderValueProvider.getAsDouble() * TICK_TO_DISTANCE;
        }

        /**
         * <p>
         *      Calculates and returns the displacement of the left parallel dead wheel using the provided ticks to distance unit ratio.
         * <p/>
         * @return The displacement of the left parallel dead wheel.
         * */
        public double getRightFrontWheelDistance() {
            return rightFrontEncoderValueProvider.getAsDouble() * TICK_TO_DISTANCE;
        }

        /**
         * <p>
         *      Calculates and returns the displacement of the right parallel dead wheel using the provided ticks to distance unit ratio.
         * <p/>
         * @return The displacement of the right parallel dead wheel.
         * */
        public double getLeftBackWheelDistance() {
            return leftBackEncoderValueProvider.getAsDouble() * TICK_TO_DISTANCE;
        }

        /**
         * <p>
         *      Calculates and returns the displacement of the right parallel dead wheel using the provided ticks to distance unit ratio.
         * <p/>
         * @return The displacement of the right parallel dead wheel.
         * */
        public double getRightBackWheelDistance() {
            return rightBackEncoderValueProvider.getAsDouble() * TICK_TO_DISTANCE;
        }
    }

    private final MeasurementProvider measurementProvider;

    private Pose2D currPose;

    private double x;
    private double y;
    private double theta;

    private double thetaOffset;

    private final double TRACK_WIDTH;

    private double previousLeftFront = 0;
    private double previousRightFront = 0;
    private double previousLeftBack = 0;
    private double previousRightBack = 0;

    private Logger logger = null;
    private Distance.DistanceUnit distanceUnitOfMeasurement = null;

    /**
     * <p>Creates a position tracker at a specified position.<p/>
     * @param initialPosition The initial coordinates of the robot in the given distance unit.
     * @param initialHeading The initial heading of the robot in Degrees.
     * @param trackWidth The distance between the centers of the two parallel dead wheels - used for heading calculation.
     * @param measurementProvider The handler for encoder measurement updates.
     * */
    public MecanumKinematics(
            SmartPoint initialPosition,
            double initialHeading,
            double trackWidth,
            MeasurementProvider measurementProvider
    ) {
        x = initialPosition.getX().getRawValue();
        y = initialPosition.getY().getRawValue();
        TRACK_WIDTH = trackWidth;
        theta = Math.toRadians(MathUtil.shiftAngle(initialHeading, 0));
        currPose = new Pose2D(x, y, theta);
        distanceUnitOfMeasurement = initialPosition.getUnitOfMeasurement();
        this.measurementProvider = measurementProvider;
    }

    /**
     * <p>Creates a position tracker at the default starting position; (0,0).<p/>
     * @param trackWidth The distance between the centers of the two parallel dead wheels - used for heading calculation.
     * @param distanceUnitOfMeasurement The unit of measurement to be used for coordinates.
     * @param measurementProvider The handler for encoder measurement updates.
     * */
    public MecanumKinematics(
            double trackWidth,
            Distance.DistanceUnit distanceUnitOfMeasurement,
            MeasurementProvider measurementProvider
    ) {
        this(
                new SmartPoint(distanceUnitOfMeasurement, 0, 0),
                0,
                trackWidth,
                measurementProvider
        );
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
        return currPose;
    }

    /**
     * <p>Updates the measurements and calculates the position and heading estimate.<p/>
     * <p>
     *     Heading is calculated using {@code θ' = θ + (ΔR - ΔL) / d},
     *     where {@code θ} the previous heading, where {@code θ'} the current heading,
     *     where {@code ΔR} the displacement of the right parallel dead wheel,
     *     where {@code ΔL} the displacement of the left parallel dead wheel,
     *     where {@code d} the encoder width (the distance between the centers of the two parallel dead wheels)
     * <p/>
     * */
    @Override
    public void update() {

        // Caching cycle value
        double cLFront = measurementProvider.getLeftFrontWheelDistance();
        double cRFront = measurementProvider.getRightFrontWheelDistance();
        double cLBack = measurementProvider.getLeftBackWheelDistance();
        double cRBack = measurementProvider.getRightBackWheelDistance();

        double dLFront = cLFront - previousLeftFront;
        double dRFront = cRFront - previousRightFront;
        double dLBack = cLBack - previousLeftBack;
        double dRBack = cRBack - previousRightBack;

        double dC = (dLFront + dRFront + dLBack + dRBack) / 4;

        double dTheta = (dRBack + dRFront - dLBack - dLFront) / (4 * TRACK_WIDTH);

        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        double dForward = dC ;
        double dStrafe = (dLBack + dRFront - dLFront - dRBack) / 4 ;

        double dx = (dStrafe * sin) + (dForward * cos);
        double dy = (dStrafe * cos) + (-dForward * sin);

        x += dx;
        y += dy;

        theta += dTheta;
        theta = Math.toRadians(MathUtil.shiftAngle(Math.toDegrees(theta), 0));

        previousLeftFront = cLFront;
        previousRightFront = cRFront;
        previousLeftBack = cLBack;
        previousRightBack = cRBack;

        currPose = new Pose2D(x, y, theta);
    }

    /**
     * <p>
     *    Attaches a logger to this instance to record debug values.
     * <p/>
     * */
    public void attachLogger(Logger logger) {
        this.logger = logger;
    }

    /**
     * <p>
     *    Runs the debug actions, such as logging, of this system.
     * <p/>
     * */
    public void debug() {
        if (logger != null) {
            logger.logValue("robotX", x);
            logger.logValue("robotY", y);
            logger.logValue("robotHeading", theta);
        }
    }
}
