package com.github.bouyio.cyancore.localization;

import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.MathUtil;

import java.util.function.DoubleSupplier;

public class ThreeDeadWheelOdometry implements PositionProvider {

    public static class MeasurementProvider {
        public final DoubleSupplier perpendicularEncoderValueProvider;
        public final DoubleSupplier leftParallelEncoderValueProvider;
        public final DoubleSupplier rightParallelEncoderValueProvider;
        private final double TICK_TO_DISTANCE;

        public MeasurementProvider(
                DoubleSupplier perpendicularEncoderValueProvider,
                DoubleSupplier leftParallelEncoderValueProvider,
                DoubleSupplier headingProvider,
                double ticksToDistance
        ) {
            this.perpendicularEncoderValueProvider = perpendicularEncoderValueProvider;
            this.leftParallelEncoderValueProvider = leftParallelEncoderValueProvider;
            this.rightParallelEncoderValueProvider = headingProvider;
            TICK_TO_DISTANCE = ticksToDistance;
        }

        public double getPerpendicularWheelDistance() {
            return perpendicularEncoderValueProvider.getAsDouble() * TICK_TO_DISTANCE;
        }

        public double getLeftParallelWheelDistance() {
            return leftParallelEncoderValueProvider.getAsDouble() * TICK_TO_DISTANCE;
        }
        
        public double getRightParallelWheelDistance() {
            return rightParallelEncoderValueProvider.getAsDouble() * TICK_TO_DISTANCE;
        }
    }

    private final MeasurementProvider measurementProvider;

    private Pose2D currPose;

    private double x;
    private double y;
    private double theta;

    private double thetaOffset;

    private final double ENCODER_WIDTH;

    private double previousPerpendicular = 0;
    private double previousLeftParallel = 0;
    private double previousRightParallel = 0;

    private Logger logger = null;
    private Distance.DistanceUnit distanceUnitOfMeasurement = null;

    public ThreeDeadWheelOdometry(
            SmartPoint initialPosition,
            double initialHeading,
            double encoderWidth,
            MeasurementProvider measurementProvider
            ) {
        x = initialPosition.getX().getRawValue();
        y = initialPosition.getY().getRawValue();
        ENCODER_WIDTH = encoderWidth;
        thetaOffset = Math.toRadians(initialHeading);
        currPose = new Pose2D(x, y, thetaOffset);
        distanceUnitOfMeasurement = initialPosition.getUnitOfMeasurement();
        this.measurementProvider = measurementProvider;
    }

    @Override
    public Pose2D getPose() {
        return currPose;
    }

    @Override
    public void update() {

        double dPerpendicular =
                 measurementProvider.getPerpendicularWheelDistance() - previousPerpendicular;
        double dLParallel = measurementProvider.getLeftParallelWheelDistance() - previousLeftParallel;
        double dRParallel = measurementProvider.getRightParallelWheelDistance() - previousRightParallel;

        double dTheta = (dRParallel - dLParallel) / ENCODER_WIDTH;
        double dParallel = (dRParallel + dLParallel) / 2;

        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        double dy = (dPerpendicular * sin) + (dParallel * sin);
        double dx = (-dPerpendicular * cos) + (dParallel * cos);

        x += dx;
        y += dy;

        theta += dTheta;
        theta = Math.toRadians(MathUtil.shiftAngle(Math.toDegrees(theta), 0));

        previousPerpendicular = dPerpendicular;
        previousLeftParallel = dLParallel;
        previousRightParallel = dRParallel;

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
