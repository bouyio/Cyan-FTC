package com.github.bouyio.cyancore.pathing;

import com.github.bouyio.cyancore.localization.PositionProvider;
import com.github.bouyio.cyancore.pathing.engine.PathFollower;

import java.util.ArrayList;
import java.util.List;

/**
 * <p>A simple way to sequence paths.<p/>
 * @see Path
 * @see PathFollower
 * */
public class PathSequence {
    List<Path> paths = new ArrayList<>();

    private int currentPathIndex = 0;

    private final double errorThreshold;
    private final PositionProvider positionProvider;

    /**
     * <p>
     *     Creates a sequence with the provided paths with the given order and manually set path
     *     error tolerance.
     * <p/>
     * @param posProvider The {@code PositionProvider} of the robot used for determining whether a path is finished.
     * @param errorThreshold The error tolerance of the paths used for determining whether a path is finished.
     * @param sequencePaths The paths to be sequenced in the given order.
     * */
    public PathSequence(PositionProvider posProvider, double errorThreshold, Path... sequencePaths) {
        this.errorThreshold = errorThreshold;
        positionProvider = posProvider;

        for (Path p : sequencePaths) {
            p.setMinimumPathError(errorThreshold);
            paths.add(p);
        }
    }

    /** @return The currently focused path of the sequence.*/
    private Path getCurrentPath() { return paths.get(currentPathIndex); }

    /**
     * <p>
     *     Updates the sequence and returns the currently focused path.
     * </p>
     *
     * <ul>
     *   <li>
     *       If the currently focused path is finished, the robot's distance to its last point is less
     *       than the specified error threshold, it selects the next path.
     *   </il>
     *   <li>
     *       If the sequence is finished, it returns null.
     *   </il>
     * </ul>
     *
     * @return The currently selected path or null if the sequence is finished.
     * @implNote Calls {@link PositionProvider#update()}.
     * */
    public Path nextPathUpdate() {
        if (currentPathIndex == paths.size() - 1) return null;

        positionProvider.update();
        if (getCurrentPath().isPathFinished(positionProvider.getPose())) {
            currentPathIndex++;
        }
        return getCurrentPath();
    }


    /**
     * <p>Adds a path to the end of the sequence.<p/>
     * @param p The path to be added.
     * */
    public void appendPath(Path p) {
        p.setMinimumPathError(errorThreshold);
        paths.add(p);
    }

    /**
     * <p>Inserts a point at specific index of the sequence.<p/>
     * @param index The index where the path is to be inserted.
     * @param p The path to be inserted.
     * */
    public void insertPath(int index, Path p) {
        p.setMinimumPathError(errorThreshold);
        paths.add(index, p);
    }


    /**
     * <p>Resets the sequence in order to be reused.<p/>
     * */
    public void reset() {
        currentPathIndex = 0;
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
