package com.github.bouyio.cyancore.pathing;

import com.github.bouyio.cyancore.localization.PositionProvider;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Point;
import com.github.bouyio.cyancore.util.MathUtil;

import static java.lang.Math.*;

import java.util.ArrayList;
import java.util.List;

/**
 * <p>
 *     Calculates the optimal point of a path to be followed.
 *    Implementation of the circle line intersection - pure pursuit algorithm used for smooth path following.
 * <p/>
 * @see Path
 * @see PathFollower
 * @see <a href="https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf">"Implementation of the Pure Pursuit Path Tracking Algorithm" paper</a>
 * */
public class CircleLineIntersectionCalculator {
    private final double lookAheadDistance;
    private final PositionProvider posProvider;

    private Path targetPath;

    private double differenceThreshold = 0.003;

    private final double admissiblePointError;

    // ----SYSTEM VERSION INFO---
    private final String SYSTEM_VERSION = "1.0";
    private final String SYSTEM_NAME = "CLI_CALC";
    public String getSystemVersion() {return SYSTEM_VERSION;}
    public String getSystemName() {return SYSTEM_NAME;}

    // ----DEBUG FIELDS----
    private Logger logger = null;
    private int dbgPointSolutions = 0;
    private double dbgDiscriminant = -1;
    private double dbgSol1X = Double.MAX_VALUE;
    private double dbgSol1Y = Double.MIN_VALUE;
    private double dbgSol2X = Double.MAX_VALUE;
    private double dbgSol2Y = Double.MIN_VALUE;
    private int dbgSegmentID = 0;


    /**
     * <p>Creates a circle line intersection calculator instance.<p/>
     * @param positionProvider The robot's localization system.
     * @param radius The length of the circle's radius used for the circle line intersection calculation.
     * @param admissibleError The minimum value that could be considered error.
     * */
    public CircleLineIntersectionCalculator(PositionProvider positionProvider, double radius, double admissibleError) {
        lookAheadDistance = radius;
        posProvider = positionProvider;
        admissiblePointError = admissibleError;
    }

    /**
     * <p>Sets the path used by the system to calculated the next point.<p/>
     * @param path The target path.
     * */
    public void setTargetPath(Path path) {
        this.targetPath = path;
        targetPath.setMinimumPathError(admissiblePointError);
    }

    /**
     * <p>Sets the distance between two points required for them to be considered as separate points.<p/>
     * @param threshold Point difference threshold.
     * */
    public void setDifferenceThreshold(double threshold) {
        differenceThreshold = threshold;
    }

    /**
     * <p>
     *     Calculates all the possible points of segment that can be followed using the circle line intersection algorithm.
     * <p/>
     *
     * @param point1 The first point of a segment.
     * @param point2 The last point of a segment.
     * @return A list of all followable points.
     * @throws ArithmeticException If the discriminant is negative - The circle does not intersect with any of the segment's points.
     * @implNote Calls {@link PositionProvider#update()}.
     * */
    private List<Point> calculateCircleLineIntersection(Point point1, Point point2) {
        posProvider.update();
        Point circleCenter = posProvider.getPose().toPoint();

        point1 = new Point(
                abs(point1.getCoordinates().getX() - point2.getCoordinates().getX()) < differenceThreshold ?
                        point1.getCoordinates().getX() + differenceThreshold : point1.getCoordinates().getX(),

                abs(point1.getCoordinates().getY() - point2.getCoordinates().getY()) < differenceThreshold ?
                        point1.getCoordinates().getY() + differenceThreshold : point1.getCoordinates().getY()
        );

        // Components of the quadratic equation.
        double m1 = (point2.getCoordinates().getY() - point1.getCoordinates().getY()) /
                (point2.getCoordinates().getX() - point1.getCoordinates().getX());

        double quadraticA = 1 + pow(m1, 2);

        // The first point's coordinates relative to the circle center.
        double x1 = point1.getCoordinates().getX() - circleCenter.getCoordinates().getX();
        double y1 = point1.getCoordinates().getY() - circleCenter.getCoordinates().getY();

        double quadraticB = (2 * m1 * y1) - (2 * pow(m1, 2) * x1);

        double quadraticC = ((pow(m1, 2) * pow(x1, 2))) - (2*y1*m1*x1) + pow(y1, 2) - pow(lookAheadDistance, 2);

        double discriminant = pow(quadraticB, 2) - 4 * quadraticA * quadraticC;
        dbgDiscriminant = discriminant;

        if (discriminant < 0) throw new ArithmeticException("Discriminant must be non-negative.");

        List<Point> solutions = new ArrayList<>();

        // The area of the section that the solutions must be in.
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

    /**
     * <p>
     *     Chooses the optimal point of the current segment.
     *     Determines when to switch segments.
     *     If the robot is far of the current segment it chooses the nearest declared point of the {@link Path}.
     * <p/>
     *
     * @return The optimal point of the path to be followed.
     * @implNote Calls {@link PositionProvider#update()}.
     * */
    public Point getTargetPoint() {
        posProvider.update();

        if(targetPath.isPathFinished(posProvider.getPose())) return null;

        Point[] currentSegment = targetPath.getCurrentSegment();

        Point nearestNextPoint = targetPath.getClosestNextPoint(posProvider.getPose());

        // Logic for if the focused segment should be switched.

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

            // In case the discriminant is negative.
            solutions = new ArrayList<>();
            solutions.add(targetPath.getClosestPoint(posProvider.getPose()));
        }

        // Finding the best solution.
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


    /**
     * <p>
     *    Attaches a logger to this instance to record debug values.
     * <p/>
     * */
    public void attachLogger(Logger l) {
        logger = l;
    }

    /**
     * <p>
     *    Runs the debug actions, such as logging, of this system.
     * <p/>
     * */
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
