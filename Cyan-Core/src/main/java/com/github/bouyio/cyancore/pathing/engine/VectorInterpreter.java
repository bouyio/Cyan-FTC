package com.github.bouyio.cyancore.pathing.engine;

import com.github.bouyio.cyancore.geomery.Pose2D;

/**
 * <p>
 *     This is an interface used in between the PathFollower and the robot's motors.
 *     Should be implemented when support for a particular drivetrain is needed.
 *     Any drivetrain theoretically can be made compatible with this interface.
 * </p>
 * */
public interface VectorInterpreter {

    /**
     * <p>
     *     Updates the robot's error from the target and calculates the motor powers.
     * </p>
     * @param desiredPose The error of the robot from the target in x and y axis as well as its heading error.
     * */
    void process(Pose2D desiredPose);

    /**
     * <p>
     *     Requests the robot to stop any drivetrain movement.
     * </p>
     * */
    void stop();

    /**
     * <p>
     *     Returns the calculated power inputs to be applied to the drivetrain motors.
     * </p>
     * <p>
     *     Note: There is no standard for which motor each index corresponds.
     *     To avoid confusion and possible OutOfBounds errors, please refer to the specific implementation.
     * </p>
     * @return The power inputs to be applied to the drivetrain motors.
     * */
    double[] getMotorInputs();
}
