package com.github.bouyio.cyancore.pathing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import com.github.bouyio.cyancore.geomery.Point;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.util.Distance;

/**
 * A sequence of points meant to be used as a path for the circle line intersection algorithm.
 * It provides a much smoother and more coherent path than {@link PointSequence}.
 * To follow such path a follower such as {@link PathFollower} is needed.
 * Optimized with improved bounds checking and efficient segment advancement.
 *
 * @see Point
 * @see CircleLineIntersectionCalculator
 * @see PathFollower
 * @author Bouyio (https://github.com/bouyio)
 * @author Gvol (https://github.com/Gvolexe)
 */
public class Path {
    private Distance.DistanceUnit distanceUnitOfMeasurement = null;
    private final List<Point> pathPoints;

    private int segmentIndex = 0;
    private boolean isOnLastSegment = false;

    private double admissibleError = 0.05;

    /**
     * <p>Creates a path with the points in the order they are given.<p/>
     * @param points The points of the path in the given order.
     * */
    public Path(Point... points) {
        pathPoints = new ArrayList<>();
        Collections.addAll(pathPoints, points);
    }

    /**
     * <p>
     *     Sets the minimum distance that can be considered error by the path.
     *     Mainly used for determining whether the path is finished.
     * <p/>
     * @param error The minimum admissible path error.
     * */
    public void setMinimumPathError(double error) {
        admissibleError = error;
    }

    public void setDistanceUnitOfMeasurement(Distance.DistanceUnit unit) {
        distanceUnitOfMeasurement = unit;
    }

    public Distance.DistanceUnit getDistanceUnitOfMeasurement() {
        return distanceUnitOfMeasurement;
    }

    /**
     * <p>Returns the segment (area between two points) the robot is estimated to be in.<p/>
     * @return A array of two points representing the current segment.
     * */
    public Point[] getCurrentSegment() {
        // Optimized: Add bounds checking and more efficient logic
        if (pathPoints.isEmpty()) {
            throw new IllegalStateException("Path has no points");
        }
        
        if (segmentIndex >= pathPoints.size() - 1) {
            // On last segment, return last two points
            int lastIndex = pathPoints.size() - 1;
            return new Point[] {pathPoints.get(Math.max(0, lastIndex - 1)), pathPoints.get(lastIndex)};
        }

        return new Point[] {pathPoints.get(segmentIndex), pathPoints.get(segmentIndex + 1)};
    }

    /**@return The index id of the segment.*/
    public int getSegmentIndex() {
        return segmentIndex;
    }


    /**
     * <p>Changes the segment that is estimated to be in to the next segment.<p/>
     * */
    public void nextSegment() {
        // Optimized: More efficient segment advancement logic
        if (isOnLastSegment || pathPoints.isEmpty()) return;

        segmentIndex++;
        if (segmentIndex >= pathPoints.size() - 1) {
            isOnLastSegment = true;
            segmentIndex = Math.max(0, pathPoints.size() - 2);  // Ensure valid index
        }
    }

    /**@return The amount of declared points in the path.*/
    public int getPathLength() {
        return pathPoints.size();
    }

    /**
     * <p>Checks if the robot has finished the path using its position.<p/>
     * @param pose The current pose of the robot.
     * */
    public boolean isPathFinished(Pose2D pose) {
        return pathPoints.get(getPathLength() - 1).getDistanceFrom(pose) < admissibleError && isOnLastSegment;
    }

    /**
     * <p>Resets the path (the current segment estimation) to be reused.<p/>
     * */
    public void reset() {
        segmentIndex = 0;
        isOnLastSegment = false;
    }

    /**
     * <p>Calculates the closest declared point of the path using the robot's position.<p/>
     * @param pose The nearest point.
     * */
    public Point getClosestPoint(Pose2D pose) {
        Point nearest = null;
        for (Point p : pathPoints) {

            if (nearest == null) {
                nearest = p;
                continue;
            }

            // This 0.003 should be configurable.
            // Oh well. ¯\_(ツ)_/¯
            if (nearest.getDistanceFrom(pose) > p.getDistanceFrom(pose) &&
                    (Math.abs(nearest.getDistanceFrom(pose) - p.getDistanceFrom(pose))) > 0.003) nearest = p;

        }

        return nearest;
    }

    /**
     * <p>Calculates the closest declared point that doesn't belong to the segment the robot is estimated to be of the path using the robot's position.<p/>
     * @param pose The nearest point.
     * */
    public Point getClosestNextPoint(Pose2D pose) {
        Point nearest = null;

        int startingIndex = segmentIndex + 1 < getPathLength() ? segmentIndex + 2 : segmentIndex;


        for (int i = startingIndex; i < getPathLength(); i++) {
            Point p = pathPoints.get(i);

            if (nearest == null) {
                nearest = p;
                continue;
            }

            // This 0.003 should be configurable.
            // Oh well. ¯\_(ツ)_/¯
            if (nearest.getDistanceFrom(pose) > p.getDistanceFrom(pose) &&
                    (Math.abs(nearest.getDistanceFrom(pose) - p.getDistanceFrom(pose))) > 0.003) nearest = p;

        }

        return nearest;
    }

    /**
     * <p>
     *     Creates a copy of the path object.
     * </p>
     * @return The copy of the path
     * */
    public Path copy() {

        Point[] pathPoints = new Point[getPathLength()];
        pathPoints = this.pathPoints.toArray(pathPoints);

        return new Path(
                pathPoints
        );
    }

    /**
     * <p>
     *     Creates a copy of the path object with its points arranged in reverse order.
     * </p>
     * @return The copy of the path
     * */
    public Path reverse() {

        ArrayList<Point> copy = new ArrayList<>();
        Collections.copy(copy, pathPoints);
        Collections.reverse(copy);

        return new Path(
                copy.toArray(new Point[0])
        );
    }
}
