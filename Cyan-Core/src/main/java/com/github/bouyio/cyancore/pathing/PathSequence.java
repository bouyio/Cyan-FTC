package com.github.bouyio.cyancore.pathing;

import com.github.bouyio.cyancore.localization.PositionProvider;

import java.util.ArrayList;
import java.util.List;

public class PathSequence {
    List<Path> paths = new ArrayList<>();

    private int currentPathIndex = 0;

    private final double errorThreshold;
    private final PositionProvider positionProvider;

    public PathSequence(PositionProvider posProvider, double errorThreshold, Path... sequencePaths) {
        this.errorThreshold = errorThreshold;
        positionProvider = posProvider;

        for (Path p : sequencePaths) {
            p.setMinimumPathError(errorThreshold);
            paths.add(p);
        }
    }

    private Path getCurrentPath() { return paths.get(currentPathIndex); }

    public Path nextPathUpdate() {
        if (currentPathIndex == paths.size() - 1) return null;

        positionProvider.update();
        if (getCurrentPath().isPathFinished(positionProvider.getPose())) {
            currentPathIndex++;
        }
        return getCurrentPath();
    }

    public void appendPath(Path p) {
        p.setMinimumPathError(errorThreshold);
        paths.add(p);
    }

    public void insertPoint(int index, Path p) {
        p.setMinimumPathError(errorThreshold);
        paths.add(index, p);
    }


    /**
     * <p>
     *     Creates a copy of the sequence object.
     * </p>
     * @return The copy of the sequence
     * */
    public PathSequence copy() {

        Path[] seqPaths = new Path[paths.size()];

        for (int i = 0; i < paths.size(); i++) {
            seqPaths[i] = this.paths.get(i);
        }

        return new PathSequence(
                positionProvider,
                errorThreshold,
                seqPaths
        );
    }

    /**
     * <p>
     *     Creates a copy of the sequence object with its paths arranged in reverse order.
     * </p>
     * @return The copy of the sequence
     * */
    public PathSequence reverse() {

        Path[] seqPaths = new Path[paths.size()];

        for (int i = 0; i < paths.size(); i++) {
            seqPaths[i] = paths.get(paths.size() - i -1);
        }

        return new PathSequence(
                positionProvider,
                errorThreshold,
                seqPaths
        );
    }
}
