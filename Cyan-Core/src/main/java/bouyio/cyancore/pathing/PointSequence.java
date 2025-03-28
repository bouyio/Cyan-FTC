package bouyio.cyancore.pathing;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import bouyio.cyancore.geomery.Point;

public class PointSequence {
    List<Point> points = new LinkedList<>();

    private int currentPointIndex = 0;

    public PointSequence(Point... sequencePoints) {
        Collections.addAll(points, sequencePoints);
    }

    public Point getCurrentPoint() {
        return points.get(currentPointIndex);
    }

    public Point nextPoint() {
        if (currentPointIndex == points.size()) return null;

        currentPointIndex++;
        return getCurrentPoint();
    }

    public void appendPoint(Point p) {
        points.add(p);
    }

    public void insertPoint(int index, Point p) {
        points.add(index, p);
    }
}
