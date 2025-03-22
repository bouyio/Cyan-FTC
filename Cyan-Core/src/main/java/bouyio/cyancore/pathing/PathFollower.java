package bouyio.cyancore.pathing;

import bouyio.cyancore.PositionProvider;
import bouyio.cyancore.geomery.Point;
import bouyio.cyancore.util.MathUtil;

// TODO: Implement Pure pursuit
// TODO: Test point following
public class PathFollower {

    PositionProvider posProvider;
    private final double lookAheadDistance;

    public PathFollower(PositionProvider posProvider, double lookAheadDistance) {
        this.posProvider = posProvider;
        this.lookAheadDistance = lookAheadDistance;
    }

    public double[] calculatePowersToPoint(Point point) {
        posProvider.update();

        double deltaX = point.getCoordinates().getX() - posProvider.getPose().getX();
        double deltaY = point.getCoordinates().getY() - posProvider.getPose().getY();

        double distanceToPoint = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        double linearPower = distanceToPoint / (Math.abs(deltaX) + Math.abs(deltaY));

        double angleError = MathUtil.wrapAngle(Math.atan2(deltaY, deltaX) - posProvider.getPose().getTheta());
        double steeringPower = angleError / Math.PI;

        return new double[] {linearPower, steeringPower};
    }
}
