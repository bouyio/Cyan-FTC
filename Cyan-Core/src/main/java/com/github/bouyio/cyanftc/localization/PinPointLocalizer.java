package com.github.bouyio.cyanftc.localization;

import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.localization.PositionProvider;
import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyanftc.util.RcToCyanDistanceUnit;
import com.github.bouyio.cyanftc.util.RcToCyanPose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PinPointLocalizer implements PositionProvider {

    private Pose2D pose;

    private final Distance.DistanceUnit unitOfMeasurement;

    private final GoBildaPinpointDriver pinpointDriver;

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

    @Override
    public Pose2D getPose() {
        return pose;
    }

    @Override
    public void update() {
        pinpointDriver.update();
        pose = RcToCyanPose.toCyan(pinpointDriver.getPosition(), unitOfMeasurement);
    }
}
