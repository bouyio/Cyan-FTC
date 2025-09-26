package com.github.bouyio.cyancore.pathing.engine;

import com.github.bouyio.cyancore.geomery.Pose2D;

public interface VectorInterpreter {
    void process(Pose2D desiredPose);

    void stop();

    double[] getMotorInputs();
}
