package com.github.bouyio.cyancore.geomery;

import com.github.bouyio.cyancore.util.Distance;

public class SmartVector {
    private final Distance x;
    private final Distance y;
    private final Distance.DistanceUnit unitOfMeasurement;

    public SmartVector(Distance.DistanceUnit unit, double x, double y) {
        this.x = new Distance(x, unit);
        this.y = new Distance(y, unit);
        unitOfMeasurement = unit;
    }

    public Distance getX() {
        return x;
    }

    public Distance getY() {
        return y;
    }

    public Pose2D getAsPose(Distance.DistanceUnit unit) {
        return new Pose2D(x.getAs(unit), y.getAs(unit), 0);
    }

    public Pose2D getAsPose() {
        return new Pose2D(x.getAs(unitOfMeasurement), y.getAs(unitOfMeasurement), 0);
    }

    public Distance.DistanceUnit getUnitOfMeasurement() {
        return unitOfMeasurement;
    }
}
