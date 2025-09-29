package com.github.bouyio.cyancore.pathing.engine;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Loggable;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.debugger.formating.Identifier;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.util.MathUtil;

public class TankDriveVectorInterpreter implements VectorInterpreter, Loggable {

    private Logger logger = null;

    private double leftMotorInput = 0;
    private double rightMotorInput = 0;
    private final boolean reverseDriveEnabled;

    public static final int LEFT_MOTOR_INDEX_ID = 0;
    public static final int RIGHT_MOTOR_INDEX_ID = 1;

    private final String SYSTEM_NAME = "TANK_VI";
    private final String SYSTEM_VERSION = "1.0";

    public String getSystemName() { return SYSTEM_NAME; }
    public String getSystemVersion() { return SYSTEM_VERSION; }

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

    @Override
    public void attachLogger(Logger logger) {
        this.logger = logger;
    }

    @Override
    public void log() {
        if (logger != null) {
            logger.record(new DebugPacket<>(new Identifier(SYSTEM_NAME), SYSTEM_VERSION));
            logger.logValue("Left Power", leftMotorInput);
            logger.logValue("Right Power", rightMotorInput);
            logger.logValue("Reverse Drive", reverseDriveEnabled ? "enabled" : "disabled");
        }

    }
}
