package com.github.bouyio.cyancore.pathing.engine;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Loggable;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.debugger.formating.Identifier;
import com.github.bouyio.cyancore.geomery.Pose2D;

/**
 * <p>
 *     This class is a bridge between the PathFollower and any Mecanum drivetrain.
 *     Also supports the standardised debugging interface of the Loggable System.
 * </p>
 * @see VectorInterpreter
 * @see PathFollower
 * */
public class MecanumDriveVectorInterpreter implements VectorInterpreter, Loggable {

    public enum MecanumReverseSideParameters {
        LEFT(-1), RIGHT(1);
        MecanumReverseSideParameters(int sign) {
            this.sign = sign;
        }
        public final int sign;
    }

    private Logger logger = null;

    private double[] motorInputs = new double[] {0, 0, 0, 0};

    private double dbgXPower = 0;
    private double dbgYPower = 0;

    private final MecanumReverseSideParameters reverseSide;

    public static final int LEFT_FRONT_MOTOR_ID = 0;
    public static final int LEFT_BACK_MOTOR_ID = 1;
    public static final int RIGHT_FRONT_MOTOR_ID = 2;
    public static final int RIGHT_BACK_MOTOR_ID = 3;

    // ----SYSTEM VERSION INFO----

    private final String SYSTEM_NAME = "MECANUM_VI";
    private final String SYSTEM_VERSION = "1.0";

    public String getSystemName() { return SYSTEM_NAME; }
    public String getSystemVersion() { return SYSTEM_VERSION; }

    // ----CONSTRUCTORS----
    /**
     * <p>
     *     Creates an instance of the system with specified options.
     * </p>
     * @param reverseSide The side of the drivetrain whose motors are set to {@code REVERSE}
     * */
    public MecanumDriveVectorInterpreter(MecanumReverseSideParameters reverseSide) {
        this.reverseSide = reverseSide;
    }

    /**
     * <p>
     *     Uses the error from the target to calculate the power to be applied to each motor.
     * </p>
     * */
    @Override
    public void process(Pose2D desiredPose) {
        double euclideanError = Math.hypot(desiredPose.getY(), desiredPose.getX());
        double normalizedX = desiredPose.getX() / euclideanError;
        double normalizedY = desiredPose.getY() / euclideanError;

        dbgYPower = normalizedY;
        dbgXPower = normalizedX;

        motorInputs[LEFT_FRONT_MOTOR_ID] = normalizedY + normalizedX * reverseSide.sign;
        motorInputs[LEFT_BACK_MOTOR_ID] = normalizedY - normalizedX * reverseSide.sign;
        motorInputs[RIGHT_FRONT_MOTOR_ID] = normalizedY - normalizedX * reverseSide.sign;
        motorInputs[RIGHT_BACK_MOTOR_ID] = normalizedY + normalizedX * reverseSide.sign;

        double max = 1;
        for (double motorInput : motorInputs) {
            max = Math.max(max, Math.abs(motorInput));
        }

        motorInputs[LEFT_FRONT_MOTOR_ID] /= max;
        motorInputs[LEFT_BACK_MOTOR_ID] /= max;
        motorInputs[RIGHT_FRONT_MOTOR_ID] /= max;
        motorInputs[RIGHT_BACK_MOTOR_ID] /= max;
    }

    /**
     * <p>
     *     Stops the drivetrains - Sets the power to be applied to each motor to 0.
     * </p>
     * */
    @Override
    public void stop() {
        motorInputs[LEFT_FRONT_MOTOR_ID] = 0;
        motorInputs[LEFT_BACK_MOTOR_ID] = 0;
        motorInputs[RIGHT_FRONT_MOTOR_ID] = 0;
        motorInputs[RIGHT_BACK_MOTOR_ID] = 0;
    }

    /**
     * <p>
     *     Returns the processed powers to be applied to the motors of the drivetrain.
     *     Each motor is represented by certain index:
     *     <ul>
     *        <li>Left Front Motor - 0</li>
     *        <li>Left Back Motor - 1</li>
     *        <li>Right Front Motor - 2</li>
     *        <li>Right Back Motor - 3</li>
     *     </ul>
     * </p>
     *
     * @apiNote To avoid mistakes and possible errors, please do not use raw index. Use the constants in this class that represent the motor IDs.
     * @return The processed powers to be applied to the motors.
     *
     * */
    @Override
    public double[] getMotorInputs() {
        return motorInputs;
    }

    /**
     * <p>
     *      Sets the logger instance for this instance of the system.
     * </p>
     * @param logger The logger.
     * */
    @Override
    public void attachLogger(Logger logger) {
        this.logger = logger;
    }

    /**
     * <p>
     *     Logs debug info to the logger assigned to this instance of system.
     *     If there is no logger assigned, the logging is skipped.
     * </p>
     * */
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
