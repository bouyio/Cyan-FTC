package bouyio.cyancore.pathing;

import bouyio.cyancore.PositionProvider;
import bouyio.cyancore.debugger.Logger;
import bouyio.cyancore.geomery.Point;
import bouyio.cyancore.util.MathUtil;

import static java.lang.Math.*;

import java.util.ArrayList;
import java.util.List;

public class IntersectionV2 {
    private final double lookAheadDistance;
    private final PositionProvider posProvider;

    private Path targetPath;

    private double differenceThreshold = 0.003;

    private final double admissiblePointError;

    // ----DEBUG FIELDS----
    private Logger logger = null;
    private int dbgPointSolutions = 0;
    private double dbgDiscriminant = -1;
    private double dbgSol1X = Double.MAX_VALUE;
    private double dbgSol1Y = Double.MIN_VALUE;
    private double dbgSol2X = Double.MAX_VALUE;
    private double dbgSol2Y = Double.MIN_VALUE;
    private int dbgSegmentID = 0;


    public IntersectionV2(PositionProvider positionProvider, double radius, double admissibleError) {
        lookAheadDistance = radius;
        posProvider = positionProvider;
        admissiblePointError = admissibleError;
    }

    public void setTargetPath(Path path) {
        this.targetPath = path;
        targetPath.setMaximumPathError(admissiblePointError);
    }

    public void setDifferenceThreshold(double threshold) {
        differenceThreshold = threshold;
    }

    private List<Point> calculateCircleLineIntersection(Point point1, Point point2) {
        posProvider.update();
        Point circleCenter = posProvider.getPose().toPoint();

        point1 = new Point(
                abs(point1.getCoordinates().getX() - point2.getCoordinates().getX()) < differenceThreshold ?
                        point1.getCoordinates().getX() + differenceThreshold : point1.getCoordinates().getX(),

                abs(point1.getCoordinates().getY() - point2.getCoordinates().getY()) < differenceThreshold ?
                        point1.getCoordinates().getY() + differenceThreshold : point1.getCoordinates().getY()
        );

        double m1 = (point2.getCoordinates().getY() - point1.getCoordinates().getY()) /
                (point2.getCoordinates().getX() - point1.getCoordinates().getX());

        double quadraticA = 1 + pow(m1, 2);

        double x1 = point1.getCoordinates().getX() - circleCenter.getCoordinates().getX();
        double y1 = point1.getCoordinates().getY() - circleCenter.getCoordinates().getY();

        double quadraticB = (2 * m1 * y1) - (2 * pow(m1, 2) * x1);

        double quadraticC = ((pow(m1, 2) * pow(x1, 2))) - (2*y1*m1*x1) + pow(y1, 2) - pow(lookAheadDistance, 2);

        double discriminant = pow(quadraticB, 2) - 4 * quadraticA * quadraticC;
        dbgDiscriminant = discriminant;

        if (discriminant < 0) throw new ArithmeticException("Discriminant must be non-negative.");

        List<Point> solutions = new ArrayList<>();

        double minX = min(point1.getCoordinates().getX(), point2.getCoordinates().getX());
        double maxX = max(point1.getCoordinates().getX(), point2.getCoordinates().getX());
        double minY = min(point1.getCoordinates().getY(), point2.getCoordinates().getY());
        double maxY = max(point1.getCoordinates().getY(), point2.getCoordinates().getY());

        // Calculation of the first solution.

        double xRoot1 = (-quadraticB + sqrt(discriminant) / (2 * quadraticA));
        double yRoot1 = m1 * (xRoot1 - x1) + y1;

        xRoot1 += circleCenter.getCoordinates().getX();
        yRoot1 += circleCenter.getCoordinates().getY();


        dbgPointSolutions = 0;
        dbgSol1X = xRoot1;
        dbgSol1Y = yRoot1;

        if (MathUtil.isValueInRange(minX, maxX, xRoot1) && MathUtil.isValueInRange(minY, maxY, yRoot1)) {
            solutions.add(new Point(xRoot1, yRoot1));
            dbgPointSolutions++;
        }

        // Calculation of the second solution.

        double xRoot2 = (-quadraticB - sqrt(discriminant) / (2 * quadraticA));
        double yRoot2 = m1 * (xRoot1 - x1) + y1;

        xRoot2 += circleCenter.getCoordinates().getX();
        yRoot2 += circleCenter.getCoordinates().getY();

        dbgSol2X = xRoot2;
        dbgSol2Y = yRoot2;

        if (MathUtil.isValueInRange(minX, maxX, xRoot2) && MathUtil.isValueInRange(minY, maxY, yRoot2)) {
            solutions.add(new Point(xRoot2, yRoot2));
            dbgPointSolutions++;
        }

        return solutions;
    }

    public Point getTargetPoint() {
        posProvider.update();

        if(targetPath.isPathFinished(posProvider.getPose())) return null;

        Point[] currentSegment = targetPath.getCurrentSegment();

        Point nearestNextPoint = targetPath.getClosestNextPoint(posProvider.getPose());

        boolean isFarFromSegment = nearestNextPoint != null &&
                currentSegment[1].getDistanceFrom(posProvider.getPose()) >
                        nearestNextPoint.getDistanceFrom(posProvider.getPose());

        boolean shouldSegmentBeSwitched = currentSegment[1].getDistanceFrom(posProvider.getPose()) <= lookAheadDistance;

        if (shouldSegmentBeSwitched || isFarFromSegment) {

            targetPath.nextSegment();
            currentSegment = targetPath.getCurrentSegment();
            dbgSegmentID = targetPath.getSegmentIndex();
        }

        List<Point> solutions;

        try {
            solutions = calculateCircleLineIntersection(currentSegment[0], currentSegment[1]);
        } catch (ArithmeticException e) {
            solutions = new ArrayList<>();
            solutions.add(targetPath.getClosestPoint(posProvider.getPose()));
        }

        Point preferredSolution = null;
        for (Point p : solutions) {

            if (p == null) continue;

            if (preferredSolution == null) {
                preferredSolution = p;
                continue;
            }

            if (p.getDistanceFrom(currentSegment[1]) > preferredSolution.getDistanceFrom(currentSegment[1])) {
                preferredSolution = p;
            }
        }

        if (preferredSolution == null) return targetPath.getClosestNextPoint(posProvider.getPose());


        return preferredSolution;
    }


    public void attachLogger(Logger l) {
        logger = l;
    }

    public void debug() {
        if (logger == null) return;

        logger.logValue("Discriminant", dbgDiscriminant);
        logger.logValue("Points Found", dbgPointSolutions);
        logger.logValue("Solution 1 X", dbgSol1X);
        logger.logValue("Solution 1 Y", dbgSol1Y);
        logger.logValue("Solution 2 X", dbgSol2X);
        logger.logValue("Solution 2 Y", dbgSol2Y);
        logger.logValue("Current Segment", dbgSegmentID);
        logger.logValue("Is Path Finished", targetPath.isPathFinished(posProvider.getPose()));
    }
}
