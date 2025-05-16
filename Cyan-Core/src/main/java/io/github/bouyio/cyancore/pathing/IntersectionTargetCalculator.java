package io.github.bouyio.cyancore.pathing;

import java.util.ArrayList;
import java.util.List;

import io.github.bouyio.cyancore.PositionProvider;
import io.github.bouyio.cyancore.debugger.DebugPacket;
import io.github.bouyio.cyancore.debugger.Logger;
import io.github.bouyio.cyancore.debugger.formating.IndexIdentifier;
import io.github.bouyio.cyancore.geomery.Point;
import io.github.bouyio.cyancore.util.MathUtil;

@Deprecated
public class IntersectionTargetCalculator {
    private final double lookAheadDistance;

    private final PositionProvider positionProvider;

    private LegacyPath targetPath;

    private int lastFoundIndex;
    private Point goalPoint;


    // ----DEBUG FIELDS----
    private Logger logger;
    private boolean isLoggerAttached = false;
    private List<DebugPacket<IndexIdentifier, String>> pointResult;
    private double dbgDiscriminant = 0;

    public IntersectionTargetCalculator(double lookAheadDistance, PositionProvider positionProvider) {
        this.lookAheadDistance = lookAheadDistance;
        this.positionProvider = positionProvider;
    }

    public void followPath() {

        if (targetPath == null) throw new NullPointerException("Targeted path cannot be null.");

        positionProvider.update();
        List<Point> pathPoints = targetPath.getPathPoints();

        int nextPointIndex = 0;

        for (int i = lastFoundIndex; i < targetPath.getPathLength(); i++) {
            Point p = pathPoints.get(i);
            nextPointIndex++;

            if (nextPointIndex == targetPath.getPathLength()) break;

            // Accounting for null index positions.
            if (p == null) continue;

            Point nextPoint = pathPoints.get(nextPointIndex);

            double dx = nextPoint.getCoordinates().getX() - p.getCoordinates().getX();
            double dy = nextPoint.getCoordinates().getY() - p.getCoordinates().getY();

            double dr = MathUtil.hypotenuse(dx, dy);

            double D = p.getCoordinates().getX() * nextPoint.getCoordinates().getY() -
                    p.getCoordinates().getY() * nextPoint.getCoordinates().getX();

            double discriminant = Math.pow(lookAheadDistance, 2) * Math.pow(dr, 2) - Math.pow(D, 2);

            boolean isSol1InRange = false;
            boolean isSol2InRange = false;

            dbgDiscriminant = discriminant;

            if (discriminant >= 0) {
                double solutionX1 = (D * dy + MathUtil.sgn(dy) * dx * Math.sqrt(discriminant)) / Math.pow(dr, 2);
                double solutionX2 = (D * dy - MathUtil.sgn(dy) * dx * Math.sqrt(discriminant)) / Math.pow(dr, 2);
                double solutionY1 = (-D * dy + MathUtil.sgn(dy) * Math.sqrt(discriminant)) / Math.pow(dr, 2);
                double solutionY2 = (-D * dy - MathUtil.sgn(dy) * Math.sqrt(discriminant)) / Math.pow(dr, 2);

                Point solutionPt1 = new Point(
                        solutionX1 + positionProvider.getPose().getX(),
                        solutionY1 + positionProvider.getPose().getY()
                );

                Point solutionPt2 = new Point(
                        solutionX2 + positionProvider.getPose().getX(),
                        solutionY2 + positionProvider.getPose().getY()
                );

                double minX = Math.min(p.getCoordinates().getX(), nextPoint.getCoordinates().getX());
                double minY = Math.min(p.getCoordinates().getY(), nextPoint.getCoordinates().getY());
                double maxX = Math.max(p.getCoordinates().getX(), nextPoint.getCoordinates().getX());
                double maxY = Math.max(p.getCoordinates().getY(), nextPoint.getCoordinates().getY());

//                isSol1InRange = MathUtil.isValueInRange(minX, maxX, solutionPt1.getCoordinates().getX()) &&
//                        MathUtil.isValueInRange(minY, maxY, solutionPt1.getCoordinates().getY());
//                isSol2InRange = MathUtil.isValueInRange(minX, maxX, solutionPt2.getCoordinates().getX()) &&
//                        MathUtil.isValueInRange(minY, maxY, solutionPt2.getCoordinates().getY());

                isSol1InRange = isOnSegment(p, nextPoint, solutionPt1);
                isSol2InRange = isOnSegment(p, nextPoint, solutionPt2);

                if (isSol1InRange && isSol2InRange)
                    goalPoint = solutionPt1.getDistanceFrom(nextPoint) < solutionPt2.getDistanceFrom(nextPoint) ?
                            solutionPt1 : solutionPt2;
                else if (isSol1InRange) goalPoint = solutionPt1;
                else  goalPoint = solutionPt2;

                if (goalPoint.getDistanceFrom(nextPoint) < nextPoint.getDistanceFrom(positionProvider.getPose())) {
                    lastFoundIndex = nextPointIndex - 1;
                    if (isLoggerAttached) pointResult.add(
                            new DebugPacket<>(
                                    new IndexIdentifier(nextPointIndex-1),
                                    "SUCCESS"
                            )
                    );
                    return;
                } else {
                    lastFoundIndex = nextPointIndex;
                }

                if (isLoggerAttached) pointResult.add(
                        new DebugPacket<>(
                                new IndexIdentifier(nextPointIndex-1),
                                "FAIL"
                        )
                );
            }
        }

        goalPoint = getNearestPoint();
    }

    private Point getNearestPoint() {
        Point nearest = null;
        for (Point p : getTarget().getPathPoints()) {
            if (nearest == null) {
                nearest = p;
                continue;
            }

            nearest = nearest.getDistanceFrom(positionProvider.getPose()) > p.getDistanceFrom(positionProvider.getPose()) ?
                    p : nearest;
        }

        return nearest;
    }

    boolean isOnSegment(Point p1, Point p2, Point test) {
        double dot = (test.getCoordinates().getX() - p1.getCoordinates().getX()) *
                (p2.getCoordinates().getX() - p1.getCoordinates().getX()) +
                (test.getCoordinates().getY() - p1.getCoordinates().getY()) *
                        (p2.getCoordinates().getY() - p1.getCoordinates().getY());
        if (dot < 0) return false;
        double squaredLength = Math.pow(p2.getCoordinates().getX() - p1.getCoordinates().getX(), 2) +
                Math.pow(p2.getCoordinates().getY() - p1.getCoordinates().getY(), 2);
        return dot <= squaredLength;
    }


    public void setTarget(LegacyPath target) {
        targetPath = target;
    }

    public LegacyPath getTarget() {
        return targetPath;
    }

    public Point getGoal() {
        return goalPoint;
    }

    public void attachLogger(Logger logger) {
        this.logger = logger;
        isLoggerAttached = true;
        pointResult = new ArrayList<>();
    }

    public void debug() {
        if (isLoggerAttached) {
            logger.logValue("Goal Point", goalPoint.toString());
            logger.logValue("Last Found Index", lastFoundIndex);
            logger.logValue("Discriminant", dbgDiscriminant);

            for (DebugPacket<IndexIdentifier, String> packet:
                 pointResult) {
                if (packet == null) continue;
                logger.record(packet);
            }

            pointResult.clear();
        }
    }
}
