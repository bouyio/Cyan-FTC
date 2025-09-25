package com.github.bouyio.cyancore.pathing.engine;

import com.github.bouyio.cyancore.geomery.Pose2D;

public class MecanumDriveVectorInterpreter implements VectorInterpreter {
 
    private double[] motorInputs = new double[] {0, 0, 0, 0};

    public static final int LEFT_FRONT_MOTOR_ID = 0;
    public static final int LEFT_BACK_MOTOR_ID = 1;
    public static final int RIGHT_FRONT_MOTOR_ID = 2;
    public static final int RIGHT_BACK_MOTOR_ID = 3;

    @Override
    public void process(Pose2D desiredPose) {
        double euclideanError = Math.hypot(desiredPose.getY(), desiredPose.getX());
        double normalizedX = desiredPose.getX() / euclideanError;
        double normalizedY = desiredPose.getY() / euclideanError;

        motorInputs[LEFT_FRONT_MOTOR_ID] = normalizedY + normalizedX;
        motorInputs[LEFT_BACK_MOTOR_ID] = normalizedY - normalizedX;
        motorInputs[RIGHT_FRONT_MOTOR_ID] = normalizedY - normalizedX;
        motorInputs[RIGHT_BACK_MOTOR_ID] = normalizedY + normalizedX;

        double max = 1;
        for (double motorInput : motorInputs) {
            max = Math.max(max, Math.abs(motorInput));
        }

        for (double motorInput : motorInputs) {
            motorInput /= max;
        }
    }

    @Override
    public void stop() {
        for (double motorInput : motorInputs) {
            motorInput = 0;
        }
    }

    @Override
    public double[] getMotorInputs() {
        return motorInputs;
    }
}
