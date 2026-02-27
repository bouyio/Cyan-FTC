package com.github.bouyio.cyanftc.localization;

import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.localization.PositionProvider;
import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.MathUtil;
import com.github.bouyio.cyanftc.util.RcToCyanDistanceUnit;
import com.github.bouyio.cyanftc.util.RcToCyanPose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * <p>
 *     Utilizes the GoBilda PinPoint localization computer to estimate the robot's position.
 * <p/>
 * @see PositionProvider
 * @see Pose2D
 * */
public class PinPointLocalizer implements PositionProvider {

    private Pose2D pose;
    private final SmartPoint startingPosition;
    private final double thetaOffset;

    private final Distance.DistanceUnit unitOfMeasurement;

    private final GoBildaPinpointDriver pinpointDriver;

    /**
     * <p>Creates a position tracker at a specified position.<p/>
     * @param initialPosition The initial position of the robot.
     * @param initialHeading The initial heading of the robot in Degrees.
     * @param pinpointDriver The handler for the GoBilda Computer interface.
     * */
    public PinPointLocalizer(
            SmartPoint initialPosition,
            double initialHeading,
            GoBildaPinpointDriver pinpointDriver
    ) {
        this.pinpointDriver = pinpointDriver;
        this.pinpointDriver.setHeading(initialHeading, AngleUnit.DEGREES);
        startingPosition = initialPosition;
        unitOfMeasurement = initialPosition.getUnitOfMeasurement();
        pose = new Pose2D(
                initialPosition.getX().getRawValue(),
                initialPosition.getY().getRawValue(),
                Math.toRadians(initialHeading)
        );
        thetaOffset = initialHeading;
    }

    // HELP, I AM STUCK IN SCHOOL DUE EXTREME SNOWFALL

    /**
     * <p>
     *      Set up parameters for the GoBilda PinPoint Driver. Including:
     *      <ul>
     *          <li>Offset of the y odometry pod</li>
     *          <li>Offset of the x odometry pod</li>
     *          <li>Encoder resolution</li>
     *          <li>Direction of the x odometry</li>
     *          <li>Direction of the y odometry</li>
     *      </ul>
     * </p>
     *
     * <p>
     *     <strong>Note:</strong> all of the parameters should be tuned to the specification of the manufacturer.
     *     For more information consult the
     *     <a href="https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf">user guide</a>.
     * </p>
     * @param centerOffSet The offset of the x and y odometry pods.
     * @param encoderResolution The encoder resolution of the odometry pods.
     * @param xDirection The direction of the x encoder.
     * @param yDirection The direction of the y encoder.
     * */
    public void pinPointSetUp(
            SmartPoint centerOffSet,
            GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution,
            GoBildaPinpointDriver.EncoderDirection xDirection,
            GoBildaPinpointDriver.EncoderDirection yDirection
    ) {
        this.pinpointDriver.setOffsets(
                centerOffSet.getX().getRawValue(),
                centerOffSet.getY().getRawValue(),
                RcToCyanDistanceUnit.toRC(centerOffSet.getUnitOfMeasurement())
        );
        this.pinpointDriver.setEncoderResolution(encoderResolution);
        this.pinpointDriver.setEncoderDirections(xDirection, yDirection);
    }

    /**
     * <p>Creates a position tracker at the default starting position; (0,0).<p/>
     * @param unitOfMeasurement The unit of measurement to be used for coordinates.
     * @param initialHeading The initial heading of the robot in Degrees.
     * @param pinpointDriver The handler for the GoBilda Computer interface.
     * */
    public PinPointLocalizer(
            Distance.DistanceUnit unitOfMeasurement,
            double initialHeading,
            GoBildaPinpointDriver pinpointDriver
    ) {
        this(
                new SmartPoint(unitOfMeasurement, 0, 0),
                initialHeading,
                pinpointDriver
        );
    }

    /**
     * <p>Formats the x, y and heading of the robot as {@link Pose2D}.<p/>
     * @return The x, y, and heading in their respective units.
     * */
    @Override
    public Pose2D getPose() {
        return pose;
    }

    /**
     * <p>Updates the position estimate.<p/>
     * */
    @Override
    public void update() {
        pinpointDriver.update();
        pose = new Pose2D(
                pinpointDriver.getPosX(RcToCyanDistanceUnit.toRC(unitOfMeasurement)) + startingPosition.getX().getRawValue(),
                pinpointDriver.getPosY(RcToCyanDistanceUnit.toRC(unitOfMeasurement)) + startingPosition.getY().getRawValue(),
                Math.toRadians(MathUtil.shiftAngle(pinpointDriver.getHeading(AngleUnit.DEGREES), thetaOffset))
        );
    }
}
