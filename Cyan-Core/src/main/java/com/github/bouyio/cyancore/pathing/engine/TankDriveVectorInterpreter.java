package com.github.bouyio.cyancore.pathing.engine;

import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.util.MathUtil;

public class TankDriveVectorInterpreter implements VectorInterpreter {

    private double leftMotorInput = 0;
    private double rightMotorInput = 0;
    private final boolean reverseDriveEnabled;

    public static final int LEFT_MOTOR_INDEX_ID = 0;
    public static final int RIGHT_MOTOR_INDEX_ID = 1;

    public TankDriveVectorInterpreter(boolean enableReverseDrive) {
        reverseDriveEnabled = enableReverseDrive;
    }

    @Override
    public void process(Pose2D desiredPose) {

        double linearPower = MathUtil.hypotenuse(desiredPose.getX(), desiredPose.getY());
        double steeringPower = desiredPose.getTheta() / Math.PI;

        if (Math.abs(desiredPose.getTheta()) > Math.PI / 2 && reverseDriveEnabled) {
            linearPower *= -1;
            steeringPower *= -0.5;
        }

        leftMotorInput = linearPower + steeringPower;
        rightMotorInput = linearPower - steeringPower;

        double denominator = Math.max(Math.max(Math.abs(leftMotorInput), Math.abs(rightMotorInput)), 1);
        leftMotorInput /= denominator;
        rightMotorInput /= denominator;
    }

    @Override
    public double[] getMotorInputs() {
        return new double[] {leftMotorInput, rightMotorInput};
    }

    @Override
    public void stop() {
        leftMotorInput = 0;
        rightMotorInput = 0;
    }
}
