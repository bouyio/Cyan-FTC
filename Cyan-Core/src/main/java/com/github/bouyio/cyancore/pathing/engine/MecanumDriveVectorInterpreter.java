package com.github.bouyio.cyancore.pathing.engine;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Loggable;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.debugger.formating.Identifier;
import com.github.bouyio.cyancore.geomery.Pose2D;

public class MecanumDriveVectorInterpreter implements VectorInterpreter, Loggable {

    private Logger logger = null;

    private double[] motorInputs = new double[] {0, 0, 0, 0};

    private double dbgXPower = 0;
    private double dbgYPower = 0;

    public static final int LEFT_FRONT_MOTOR_ID = 0;
    public static final int LEFT_BACK_MOTOR_ID = 1;
    public static final int RIGHT_FRONT_MOTOR_ID = 2;
    public static final int RIGHT_BACK_MOTOR_ID = 3;

    private final String SYSTEM_NAME = "MECANUM_VI";
    private final String SYSTEM_VERSION = "1.0";

    public String getSystemName() { return SYSTEM_NAME; }
    public String getSystemVersion() { return SYSTEM_VERSION; }

    @Override
    public void process(Pose2D desiredPose) {
        double euclideanError = Math.hypot(desiredPose.getY(), desiredPose.getX());
        double normalizedX = desiredPose.getX() / euclideanError;
        double normalizedY = desiredPose.getY() / euclideanError;

        dbgYPower = normalizedY;
        dbgXPower = normalizedX;

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

    @Override
    public void attachLogger(Logger logger) {
        this.logger = logger;
    }

    @Override
    public void log() {
        if (logger != null) {
            logger.record(new DebugPacket<>(new Identifier(SYSTEM_NAME), SYSTEM_VERSION));
            logger.logValue("Left Front Power", motorInputs[LEFT_FRONT_MOTOR_ID]);
            logger.logValue("Left Back Power", motorInputs[LEFT_BACK_MOTOR_ID]);
            logger.logValue("Right Front Power", motorInputs[RIGHT_FRONT_MOTOR_ID]);
            logger.logValue("Right Back Power", motorInputs[RIGHT_BACK_MOTOR_ID]);
            logger.logValue("X power", dbgXPower);
            logger.logValue("Y power", dbgYPower);
        }
    }
}
