package com.github.bouyio.cyancore.localization;

import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.MathUtil;

import java.util.function.DoubleSupplier;

/**
 * <p>
 *     Utilizes the gyroscope and the drive encoders of a differential driving base to determine its position.
 *     In order to calculate it, trigonometric functions are used.
 * <p/>
 * @see PositionProvider
 * @see Pose2D
 * */
public class GyroTankOdometry implements PositionProvider {

    /**
     * <p>
     *     This class is responsible for providing and updating the encoder and gyroscope
     *     measurements necessary for the Pose Tracker to function.
     *     Also, converts encoder inputs to linear distances.
     * <p/>
     * */
    public static class MeasurementProvider {
        public final DoubleSupplier leftEncoderValueProvider;
        public final DoubleSupplier rightEncoderValueProvider;
        public final DoubleSupplier angleProvider;
        private final double ticksToDistance;

        /**
         * <p>
         *     Creates a pose tracker measurement provider with specified left and right encoder
         *     sources, heading source and tick to distance conversion ratio.
         * </p>
         * @param leftEncoderValueProvider The source of the left encoder measurement.
         * @param rightEncoderValueProvider The source of the right encoder measurement.
         * @param headingProvider The source of the heading measurement.
         * @param ticksToDistance The encoder ticks to distance conversion ratio.
         * */
        public MeasurementProvider(
                DoubleSupplier leftEncoderValueProvider,
                DoubleSupplier rightEncoderValueProvider,
                DoubleSupplier headingProvider,
                double ticksToDistance
        ) {
            this.leftEncoderValueProvider = leftEncoderValueProvider;
            this.rightEncoderValueProvider = rightEncoderValueProvider;
            this.angleProvider = headingProvider;
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

    private Distance.DistanceUnit distanceUnitOfMeasurement;

    private Pose2D currPose;

    private double x;
    private double y;
    private double theta;

    private final double thetaOffset;

    private double previousLeft = 0;
    private double previousRight = 0;

    private double previousAngle = 0;
    private double deltaAngle = 0;

    private Logger logger;
    private boolean isLoggerAttached = false;

    /**
     * <p>Creates a position tracker at the default starting position; (0,0).<p/>
     * @param unitOfMeasurement The unit of measurement to be used for coordinates.
     * @param measurementProvider The handler for encoder and gyroscope measurement updates.
     */
    public GyroTankOdometry(Distance.DistanceUnit unitOfMeasurement, MeasurementProvider measurementProvider) {
        this(new SmartPoint(unitOfMeasurement, 0, 0), 0, measurementProvider);
    }

    /**
     * <p>Creates a position tracker at a specified position.<p/>
     * @param initialPosition The initial position of the robot.
     * @param initialHeading The initial heading of the robot in Degrees.
     * @param measurementProvider The handler for encoder and gyroscope measurement updates.
     * */
    public GyroTankOdometry(SmartPoint initialPosition, double initialHeading, MeasurementProvider measurementProvider) {
        x = initialPosition.getX().getRawValue();
        y = initialPosition.getY().getRawValue();
        thetaOffset = initialHeading;
        this.distanceUnitOfMeasurement = initialPosition.getUnitOfMeasurement();
        this.measurementProvider = measurementProvider;
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

        double currentLeft = measurementProvider.getLeftWheelDistance();
        double currentRight = measurementProvider.getRightWheelDistance();

        double dLeft = currentLeft - previousLeft;
        double dRight = currentRight - previousRight;

        double dC = (dLeft + dRight) / 2;

        double dX = dC * Math.cos(theta);
        double dY = dC * Math.sin(theta);

        previousLeft = currentLeft;
        previousRight = currentRight;

        x += dX;
        y += dY;
        currPose = new Pose2D(x, y, theta);
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
