package io.github.bouyio.cyancore.pathing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import io.github.bouyio.cyancore.geomery.Point;

@Deprecated
public class LegacyPath {

    List<Point> pathPoints;

    /**
     * This is not how pure pursuit works.
     * BUT YOU DON'T WANT TO SEE ME INSANE.
     * */
    private int minimumPointIndex = -1;

    public LegacyPath(Point... points) {
        pathPoints = new ArrayList<>();
        Collections.addAll(pathPoints, points);
    }

    public List<Point> getPathPoints() {
        return pathPoints;
    }

    public int getPathLength() {
        return pathPoints.size();
    }

    public int getMinimumPointIndex() {
        return minimumPointIndex;
    }

    public void reset() {
        minimumPointIndex = 0;
    }

    public void nextPoint() {
        minimumPointIndex++;
    }
}
