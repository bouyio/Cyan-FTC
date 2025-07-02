package com.github.bouyio.cyancore.localization;

import com.github.bouyio.cyancore.geomery.Pose2D;

/**
 * <p>
 *     Should be used for interfacing with any localization setup to provide an accurate pose of the robot.
 *     This should make any localization system compatible with the every system of the library requiring the robot's pose.
 * <p/>
 * @see Pose2D
 * */
public interface PositionProvider {

    /**
     * <p>Formats the estimated pose of the robot.<p/>
     * @return The pose of the robot as {@link Pose2D}.
     * */
    Pose2D getPose();

    /**
     * <p>Used for updating and estimating the current pose of the Robot.<p/>
     * */
    void update();
}
