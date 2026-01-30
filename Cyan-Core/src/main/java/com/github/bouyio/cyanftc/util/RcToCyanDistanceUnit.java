package com.github.bouyio.cyanftc.util;

import com.github.bouyio.cyancore.util.Distance;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Bidirectional converter between FTC SDK DistanceUnit and Cyan DistanceUnit enums.
 * <p>
 * Supported distance units:
 * <ul>
 *   <li>Centimeters (CM)</li>
 *   <li>Millimeters (MM)</li>
 *   <li>Inches (INCH)</li>
 *   <li>Meters (METER)</li>
 * </ul>
 * </p>
 * <p>
 *
 * @see com.github.bouyio.cyancore.util.Distance.DistanceUnit
 * @see org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
 */
public class RcToCyanDistanceUnit {

    /**
     * Converts an FTC SDK DistanceUnit to a Cyan DistanceUnit.
     *
     * @param unit The FTC SDK DistanceUnit to convert.
     * @return The equivalent Cyan DistanceUnit, or {@code null} if the unit is not supported.
     */
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

    /**
     * Converts a Cyan DistanceUnit to an FTC SDK DistanceUnit.
     *
     * @param unit The Cyan DistanceUnit to convert.
     * @return The equivalent FTC SDK DistanceUnit, or {@code null} if the unit is not supported.
     */
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
