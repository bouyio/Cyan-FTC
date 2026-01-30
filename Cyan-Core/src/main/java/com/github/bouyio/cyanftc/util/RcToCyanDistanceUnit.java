package com.github.bouyio.cyanftc.util;

import com.github.bouyio.cyancore.util.Distance;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RcToCyanDistanceUnit {
    public static Distance.DistanceUnit toCyan (DistanceUnit unit) {
        switch (unit) {
            case CM:
                return Distance.DistanceUnit.CM;
            case MM:
                return Distance.DistanceUnit.MM;
            case INCH:
                return Distance.DistanceUnit.INCH;
            case METER:
                return Distance.DistanceUnit.METER;
            default:
                return null;
        }
    }

    public static DistanceUnit toRC (Distance.DistanceUnit unit) {
        switch (unit) {
            case CM:
                return DistanceUnit.CM;
            case MM:
                return DistanceUnit.MM;
            case INCH:
                return DistanceUnit.INCH;
            case METER:
                return DistanceUnit.METER;
            default:
                return null;
        }
    }
}
