package bouyio.cyancore;

import bouyio.cyancore.geomery.Pose2D;

public interface PositionProvider {
    Pose2D getPose();

    void update();
}
