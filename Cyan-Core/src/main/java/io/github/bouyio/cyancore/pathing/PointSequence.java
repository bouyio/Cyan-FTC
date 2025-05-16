package io.github.bouyio.cyancore.pathing;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import io.github.bouyio.cyancore.geomery.Point;

/**
 * <p>A simple way to follow points in a sequential manner.<p/>
 * @see Point
 * @see PathFollower
 * */
public class PointSequence {
    List<Point> points = new LinkedList<>();

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
}
