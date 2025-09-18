package com.github.bouyio.cyancore.pathing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import com.github.bouyio.cyancore.geomery.Point;
import com.github.bouyio.cyancore.pathing.engine.PathFollower;
import com.github.bouyio.cyancore.util.Distance;

/**
 * <p>A simple way to follow points in a sequential manner.<p/>
 * @see Point
 * @see PathFollower
 * */
public class PointSequence {
    List<Point> points = new LinkedList<>();

    private Distance.DistanceUnit unitOfMeasurement = null;

    private int currentPointIndex = 0;

    /**
     * <p>Creates a sequence with the provided points with the given order.<p/>
     * @param sequencePoints The points of the sequence in the order they appear.
     * */
    public PointSequence(Point... sequencePoints) {
        Collections.addAll(points, sequencePoints);
    }

    /**@return The current point of the sequence.*/
    public Point getCurrentPoint() {
        return points.get(currentPointIndex);
    }

    /**
     * <p>Changes the current point of the sequence to next one.<p/>
     * @return The next point of the sequence.
     * */
    public Point nextPoint() {
        if (currentPointIndex == points.size() - 1) return null;

        currentPointIndex++;
        return getCurrentPoint();
    }

    /**
     * <p>Add a point to the end of the sequence.<p/>
     * @param p The point to be added.
     * */
    public void appendPoint(Point p) {
        points.add(p);
    }

    /**
     * <p>Inserts a point at specific index of the sequence.<p/>
     * @param index The index where the point is to be inserted.
     * @param p The point to be inserted.
     * */
    public void insertPoint(int index, Point p) {
        points.add(index, p);
    }

    /**
     * <p>Resets the sequence in order to be reused.<p/>
     * */
    public void reset() {
        currentPointIndex = 0;
    }

    /**<p>Sets the distance unit of measurement of this point sequence.</p>*/
    public void setUnitOfMeasurement(Distance.DistanceUnit unit) {
        unitOfMeasurement = unit;
    }

    /**@return The distance unit of measurement of this point sequence.*/
    public Distance.DistanceUnit getUnitOfMeasurement() {
        return unitOfMeasurement;
    }

    /**
     * <p>
     *     Creates a copy of the sequence object.
     * </p>
     * @return The copy of the sequence
     * */
    public PointSequence copy() {

        Point[] pathPoints = new Point[points.size()];
        pathPoints = points.toArray(pathPoints);

        return new PointSequence(
                pathPoints
        );
    }

    /**
     * <p>
     *     Creates a copy of the sequence object with its points arranged in reverse order.
     * </p>
     * @return The copy of the sequence
     * */
    public PointSequence reverse() {

        ArrayList<Point> copy = new ArrayList<>();
        Collections.copy(copy, points);
        Collections.reverse(copy);

        return new PointSequence(
                copy.toArray(new Point[0])
        );
    }
}
