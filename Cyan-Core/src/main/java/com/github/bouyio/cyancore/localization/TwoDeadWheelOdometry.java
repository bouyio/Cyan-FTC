package com.github.bouyio.cyancore.localization;

import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.geomery.Vector2D;
import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.MathUtil;

import java.util.function.DoubleSupplier;

public class TwoDeadWheelOdometry implements PositionProvider {

    public static class MeasurementProvider {
        public final DoubleSupplier perpendicularEncoderValueProvider;
        public final DoubleSupplier parallelEncoderValueProvider;
        public final DoubleSupplier angleProvider;
        private final double TICK_TO_DISTANCE;

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

        public double getPerpendicularWheelDistance() {
            return perpendicularEncoderValueProvider.getAsDouble() * TICK_TO_DISTANCE;
        }

        public double getParallelWheelDistance() {
            return parallelEncoderValueProvider.getAsDouble() * TICK_TO_DISTANCE;
        }
    }

    private final MeasurementProvider measurementProvider;

    private final Vector2D perpendicularWheelOffset;
    private final Vector2D parallelWheelOffset;

    private Pose2D currPose;

    private double x;
    private double y;
    private double theta;

    private double thetaOffset;

    private double previousPerpendicular = 0;
    private double previousParallel = 0;

    private Logger logger = null;
    private Distance.DistanceUnit distanceUnitOfMeasurement = null;

    public TwoDeadWheelOdometry(
            SmartPoint initialPosition,
            double initialHeading,
            Vector2D perpendicularWheelOffset,
            Vector2D parallelWheelOffset,
            MeasurementProvider measurementProvider
    ) {
        x = initialPosition.getX().getRawValue();
        y = initialPosition.getY().getRawValue();
        thetaOffset = Math.toRadians(initialHeading);
        currPose = new Pose2D(x, y, thetaOffset);
        distanceUnitOfMeasurement = initialPosition.getUnitOfMeasurement();
        this.parallelWheelOffset = parallelWheelOffset;
        this.perpendicularWheelOffset = perpendicularWheelOffset;
        this.measurementProvider = measurementProvider;
    }

    @Override
    public Pose2D getPose() {
        return currPose;
    }

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

    public void attachLogger(Logger logger) {
        this.logger = logger;
    }

    public void debug() {
        if (logger != null) {
            logger.logValue("robotX", x);
            logger.logValue("robotY", y);
            logger.logValue("robotHeading", theta);
        }
    }
}
