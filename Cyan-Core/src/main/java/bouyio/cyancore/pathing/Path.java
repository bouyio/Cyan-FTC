package bouyio.cyancore.pathing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import bouyio.cyancore.geomery.Point;
import bouyio.cyancore.geomery.Pose2D;

public class Path {
    private final List<Point> pathPoints;

    private int segmentIndex = 0;
    private boolean isOnLastSegment = false;

    private double admissibleError = 0.05;

    public Path(Point... points) {
        pathPoints = new ArrayList<>();
        Collections.addAll(pathPoints, points);
    }

    public Point[] getCurrentSegment() {
        if (segmentIndex + 1 == getPathLength())
            return new Point[] {pathPoints.get(segmentIndex - 1), pathPoints.get(segmentIndex)};

        return new Point[] {pathPoints.get(segmentIndex), pathPoints.get(segmentIndex + 1)};
    }

    public int getSegmentIndex() {
        return segmentIndex;
    }

    public void setMaximumPathError(double error) {
        admissibleError = error;
    }

    public void nextSegment() {
        if (isOnLastSegment) return;

        if (segmentIndex + 1 == getPathLength()) {
            isOnLastSegment = true;
            return;
        }

        segmentIndex++;
    }

    public int getPathLength() {
        return pathPoints.size();
    }

    public boolean isPathFinished(Pose2D pose) {
        return pathPoints.get(getPathLength() - 1).getDistanceFrom(pose) < admissibleError && isOnLastSegment;
    }

    public void reset() {
        segmentIndex = 0;
        isOnLastSegment = false;
    }

    public Point getClosestPoint(Pose2D pose) {
        Point nearest = null;
        for (Point p : pathPoints) {

            if (nearest == null) {
                nearest = p;
                continue;
            }

            if (nearest.getDistanceFrom(pose) > p.getDistanceFrom(pose) &&
                    (Math.abs(nearest.getDistanceFrom(pose) - p.getDistanceFrom(pose))) > 0.003) nearest = p;

        }

        return nearest;
    }

    public Point getClosestNextPoint(Pose2D pose) {
        Point nearest = null;

        int startingIndex = segmentIndex + 1 < getPathLength() ? segmentIndex + 2 : segmentIndex;


        for (int i = startingIndex; i < getPathLength(); i++) {
            Point p = pathPoints.get(i);

            if (nearest == null) {
                nearest = p;
                continue;
            }

            if (nearest.getDistanceFrom(pose) > p.getDistanceFrom(pose) &&
                    (Math.abs(nearest.getDistanceFrom(pose) - p.getDistanceFrom(pose))) > 0.003) nearest = p;

        }

        return nearest;
    }
}
