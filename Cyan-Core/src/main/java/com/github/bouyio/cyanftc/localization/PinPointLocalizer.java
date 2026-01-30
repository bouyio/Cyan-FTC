package com.github.bouyio.cyanftc.localization;

import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.localization.PositionProvider;
import com.github.bouyio.cyancore.util.Distance;
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
        this.pinpointDriver.setOffsets(
                initialPosition.getX().getRawValue(),
                initialPosition.getY().getRawValue(),
                RcToCyanDistanceUnit.toRC(initialPosition.getUnitOfMeasurement())
        );
        this.pinpointDriver.setHeading(initialHeading, AngleUnit.DEGREES);
        pose = RcToCyanPose.toCyan(pinpointDriver.getPosition());
        unitOfMeasurement = initialPosition.getUnitOfMeasurement();
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
        pose = RcToCyanPose.toCyan(pinpointDriver.getPosition(), unitOfMeasurement);
    }
}
