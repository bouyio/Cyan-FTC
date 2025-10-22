package com.github.bouyio.cyancore.pathing.engine;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Loggable;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.debugger.formating.Identifier;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.util.MathUtil;

/**
 * <p>
 *     This class is a bridge between the PathFollower and any Differential / Tank drivetrain.
 *     Also supports the standardised debugging interface of the Loggable System.
 * </p>
 * @see VectorInterpreter
 * @see PathFollower
 * */
public class TankDriveVectorInterpreter implements VectorInterpreter, Loggable {

    public enum TankReverseSideParameters {
        LEFT(1), RIGHT(-1);
        TankReverseSideParameters(int sign) {
            this.sign = sign;
        }
        public final int sign;
    }

    private Logger logger = null;

    private double leftMotorInput = 0;
    private double rightMotorInput = 0;
    private final boolean reverseDriveEnabled;
    private final TankReverseSideParameters reverseSide;

    public static final int LEFT_MOTOR_INDEX_ID = 0;
    public static final int RIGHT_MOTOR_INDEX_ID = 1;

    private final String SYSTEM_NAME = "TANK_VI";
    private final String SYSTEM_VERSION = "1.0";

    public String getSystemName() { return SYSTEM_NAME; }
    public String getSystemVersion() { return SYSTEM_VERSION; }

    /**
     * <p>
     *     Creates an instance of the system with specified options.
     * </p>
     * @param enableReverseDrive The option of whether the system should use reverse driving when possible
     * @param reverseSide The side of the drivetrain whose motors are set to {@code REVERSE}
     * */
    public TankDriveVectorInterpreter(boolean enableReverseDrive, TankReverseSideParameters reverseSide) {
        reverseDriveEnabled = enableReverseDrive;
        this.reverseSide = reverseSide;
    }

    /**
     * <p>
     *     Uses the x, y and heading error from the target to calculate the power to be applied to each motor.
     * </p>
     * */
    @Override
    public void process(Pose2D desiredPose) {

        double linearPower = MathUtil.hypotenuse(desiredPose.getX(), desiredPose.getY());
        double steeringPower = desiredPose.getTheta() / Math.PI;

        if (Math.abs(desiredPose.getTheta()) > Math.PI / 2 && reverseDriveEnabled) {
            linearPower *= -1;
            steeringPower *= -0.5;
        }

        leftMotorInput = linearPower + steeringPower * reverseSide.sign;
        rightMotorInput = linearPower - steeringPower * reverseSide.sign;

        double denominator = Math.max(Math.max(Math.abs(leftMotorInput), Math.abs(rightMotorInput)), 1);
        leftMotorInput /= denominator;
        rightMotorInput /= denominator;
    }

    /**
     * <p>
     *     Returns the processed powers to be applied to the motors of the drivetrain.
     *     Each motor is represented by certain index:
     *     <ul>
     *        <li>Left Motor - 0</li>
     *        <li>Right Motor - 1</li>
     *     </ul>
     * </p>
     *
     * @apiNote To avoid mistakes and possible errors, please do not use raw index. Use the constants in this class that represent the motor IDs.
     * @return The processed powers to be applied to the motors.
     *
     * */
    @Override
    public double[] getMotorInputs() {
        return new double[] {leftMotorInput, rightMotorInput};
    }

    /**
     * <p>
     *     Stops the drivetrains - Sets the power to be applied to each motor to 0.
     * </p>
     * */
    @Override
    public void stop() {
        leftMotorInput = 0;
        rightMotorInput = 0;
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
            logger.logValue("Left Power", leftMotorInput);
            logger.logValue("Right Power", rightMotorInput);
            logger.logValue("Reverse Drive", reverseDriveEnabled ? "enabled" : "disabled");
        }

    }
}
