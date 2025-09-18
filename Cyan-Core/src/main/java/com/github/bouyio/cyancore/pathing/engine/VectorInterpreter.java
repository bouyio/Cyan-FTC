package com.github.bouyio.cyancore.pathing.engine;

import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.Vector2D;

public interface VectorInterpreter {
    void process(Pose2D desiredPose);

    void stop();

    double[] getMotorInputs();
}
