package com.github.bouyio.cyancore.localization;

import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.geomery.Vector2D;
import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.MathUtil;

import java.util.function.DoubleSupplier;

/**
 * <p>
 *     Utilizes the gyroscope and the encoders of dead wheels to estimate its position.
 *     In order to calculate it, trigonometric functions are used.
 * <p/>
 * @see PositionProvider
 * @see Pose2D
 * */
public class TwoDeadWheelOdometry implements PositionProvider {

    /**
     * <p>
     *     This class is responsible for providing and updating the encoder and gyroscope
     *     measurements necessary for the Pose Tracker to function.
     *     Also, converts encoder inputs to linear distances.
     * <p/>
     * */
    public static class MeasurementProvider {
        public final DoubleSupplier perpendicularEncoderValueProvider;
        public final DoubleSupplier parallelEncoderValueProvider;
        public final DoubleSupplier angleProvider;
        private final double TICK_TO_DISTANCE;

        /**
         * <p>
         *     Creates a pose tracker measurement provider with specified left and right encoder
         *     sources, heading source and tick to distance conversion ratio.
         * </p>
         * @param perpendicularEncoderValueProvider The source of the perpendicular dead-wheel encoder measurement.
         * @param parallelEncoderValueProvider The source of the parallel dead-wheel encoder measurement.
         * @param headingProvider The source of the heading measurement.
         * @param ticksToDistance The encoder ticks to distance conversion ratio.
         * */
        public MeasurementProvider(
                DoubleSupplier perpendicularEncoderValueProvider,
                DoubleSupplier parallelEncoderValueProvider,
                DoubleSupplier headingProvider,
                double ticksToDistance
        ) {
            this.perpendicularEncoderValueProvider = perpendicularEncoderValueProvider;
            this.parallelEncoderValueProvider = parallelEncoderValueProvider;
            this.angleProvider = headingProvider;
            TICK_TO_DISTANCE = ticksToDistance;
        }

        /**
         * <p>
         *      Calculates and returns the displacement of the perpendicular dead wheel using the provided ticks
         *      to distance unit ratio.
         * <p/>
         * @return The displacement of the perpendicular dead wheel.
         * */
        public double getPerpendicularWheelDistance() {
            return perpendicularEncoderValueProvider.getAsDouble() * TICK_TO_DISTANCE;
        }

        /**
         * <p>
         *      Calculates and returns the displacement of the parallel dead wheel using the provided ticks
         *      to distance unit ratio.
         * <p/>
         * @return The displacement of the parallel dead wheel.
         * */
        public double getParallelWheelDistance() {
            return parallelEncoderValueProvider.getAsDouble() * TICK_TO_DISTANCE;
        }
    }

    private final MeasurementProvider measurementProvider;

    private Pose2D currPose;

    private double x;
    private double y;
    private double theta;

    private double thetaOffset;

    private double previousPerpendicular = 0;
    private double previousParallel = 0;

    private Logger logger = null;
    private Distance.DistanceUnit distanceUnitOfMeasurement = null;

    /**
     * <p>Creates a position tracker at a specified position.<p/>
     * @param initialPosition The initial position of the robot.
     * @param initialHeading The initial heading of the robot in Degrees.
     * @param measurementProvider The handler for encoder and gyroscope measurement updates.
     * */
    public TwoDeadWheelOdometry(
            SmartPoint initialPosition,
            double initialHeading,
            MeasurementProvider measurementProvider
    ) {
        x = initialPosition.getX().getRawValue();
        y = initialPosition.getY().getRawValue();
        thetaOffset = Math.toRadians(initialHeading);
        currPose = new Pose2D(x, y, thetaOffset);
        distanceUnitOfMeasurement = initialPosition.getUnitOfMeasurement();
        this.measurementProvider = measurementProvider;
    }

    /**
     * <p>Creates a position tracker at the default starting position; (0,0).<p/>
     * @param distanceUnitOfMeasurement The unit of measurement to be used for coordinates.
     * @param measurementProvider The handler for encoder and gyroscope measurement updates.
     */
    public TwoDeadWheelOdometry(Distance.DistanceUnit distanceUnitOfMeasurement, MeasurementProvider measurementProvider) {
        this(new SmartPoint(distanceUnitOfMeasurement, 0, 0), 0, measurementProvider);
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
     * <p>Updates the position estimate.<p/>
     * */
    @Override
    public void update() {
        theta = Math.toRadians(MathUtil.shiftAngle(measurementProvider.angleProvider.getAsDouble(),
                thetaOffset));

        double cPerpendicular = measurementProvider.getPerpendicularWheelDistance();
        double cParallel = measurementProvider.getParallelWheelDistance();

        double dPerpendicular = cPerpendicular - previousPerpendicular;
        double dParallel = cParallel - previousParallel;

        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        double dx = (dPerpendicular * sin) + (dParallel * cos);
        double dy = (-dPerpendicular * cos) + (dParallel * sin);

        x += dx;
        y += dy;

        previousPerpendicular = cPerpendicular;
        previousParallel = cParallel;

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
