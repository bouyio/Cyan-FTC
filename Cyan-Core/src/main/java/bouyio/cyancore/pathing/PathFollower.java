package bouyio.cyancore.pathing;

import bouyio.cyancore.PositionProvider;
import bouyio.cyancore.geomery.Point;

// TODO: Implement Pure pursuit
// TODO: Test sequence following
public class PathFollower {

    PositionProvider posProvider;
    private final double lookAheadDistance;

    private double angleErrorTolerance = 0;
    private double distanceErrorTolerance = 0;

    private double steeringPower = 0;
    private double linearPower = 0;

    public PathFollower(PositionProvider posProvider, double lookAheadDistance) {
        this.posProvider = posProvider;
        this.lookAheadDistance = lookAheadDistance;
    }

    public void setAngleErrorTolerance(double tolerance) {
        this.angleErrorTolerance = tolerance;
    }

    public void setDistanceErrorTolerance(double tolerance) {
        this.distanceErrorTolerance = tolerance;
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

    private double[] calculatePointError(Point point) {
        posProvider.update();

        double deltaX = point.getCoordinates().getX() - posProvider.getPose().getX();
        double deltaY = point.getCoordinates().getY() - posProvider.getPose().getY();

        double distanceToPoint = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        double angleError = Math.atan2(deltaY, deltaX) - posProvider.getPose().getTheta();

        return new double[] {distanceToPoint, angleError};
    }

    private void calculatePowers(Point point, double[] error) {

        // Not proud of this. We might need restructuring to avoid such duplication.
        double linearPower = error[0] /
                (Math.abs(point.getCoordinates().getX() - posProvider.getPose().getX()) +
                        Math.abs(point.getCoordinates().getY() - posProvider.getPose().getY()));

        double steeringPower = error[1] / Math.PI;

        this.linearPower = linearPower;
        this.steeringPower = steeringPower;
    }

    public void followPointSequence(PointSequence seq) {

        Point currentPoint = seq.getCurrentPoint();
        double[] error = calculatePointError(currentPoint);

        if (error[0] < distanceErrorTolerance && error[1] < angleErrorTolerance) {
            currentPoint = seq.nextPoint();
        }

        if (currentPoint == null) return;

        followPoint(currentPoint);
    }

    public void followPoint(Point point) {
        calculatePowers(point, calculatePointError(point));
    }

    public double[] getCalculatedPowers() {
        return new double[]{linearPower, steeringPower};
    }
}
