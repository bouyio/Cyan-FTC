package bouyio.cyancore.pathing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import bouyio.cyancore.geomery.Point;

public class Path {
    private final List<Point> pathPoints;

    private int segmentIndex = 0;
    private boolean isPathFinished = false;

    public Path(Point... points) {
        pathPoints = new ArrayList<>();
        Collections.addAll(pathPoints, points);
    }

    public Point[] getCurrentSegment() {
        return new Point[] {pathPoints.get(segmentIndex), pathPoints.get(segmentIndex + 1)};
    }

    public void nextSegment() {
        if (segmentIndex + 1 == getPathLength()) {
            isPathFinished = true;
            return;
        }

        segmentIndex++;
    }

    public int getPathLength() {
        return pathPoints.size();
    }

    public boolean isPathFinished() {
        return isPathFinished;
    }

    public void reset() {
        segmentIndex = 0;
        isPathFinished = false;
    }
}
