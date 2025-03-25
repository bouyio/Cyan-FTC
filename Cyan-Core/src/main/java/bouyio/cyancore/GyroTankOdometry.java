package bouyio.cyancore;

import bouyio.cyancore.geomery.Pose2D;
import bouyio.cyancore.util.MathUtil;

public class GyroTankOdometry implements PositionProvider {

    private Pose2D currPose;

    private double x;
    private double y;
    private double theta;

    private final double thetaOffset;

    private double previousLeft = 0;
    private double previousRight = 0;

    private double currentLeft = 0;
    private double currentRight = 0;

    public GyroTankOdometry() {
        this(0, 0, 0);
    }

    public GyroTankOdometry(double initialX, double initialY, double initialHeading) {
        x = initialX;
        y = initialY;
        thetaOffset = initialHeading;
    }

    @Override
    public Pose2D getPose() {
        return currPose;
    }

    public void updateMeasurements(double left, double right, double angle) {
        currentLeft = left;
        currentRight = right;

        theta = MathUtil.wrapAngle(Math.toRadians(angle + thetaOffset));
    }

    @Override
    public void update() {
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
}
