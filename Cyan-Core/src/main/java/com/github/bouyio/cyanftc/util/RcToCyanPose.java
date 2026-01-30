package com.github.bouyio.cyanftc.util;

import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.util.Distance;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * Utility class for converting FTC SDK Pose2D objects to Cyan Pose2D objects.
 *
 * @see com.github.bouyio.cyancore.geomery.Pose2D
 * @see org.firstinspires.ftc.robotcore.external.navigation.Pose2D
 */
public class RcToCyanPose {


    /**
     * <p>
     *     Converts an FTC SDK Pose2D to a Cyan Pose2D using centimeters as the distance unit
     *     and radians as the standard working unit of the library.
     * </p>
     *
     *
     * @param pose The FTC SDK Pose2D object to convert.
     * @return A Cyan Pose2D object with position in centimeters and heading in radians.
     * @throws NullPointerException if pose is null
     */
    public static Pose2D toCyan(org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose) {
        return new Pose2D(
                pose.getX(DistanceUnit.CM),
                pose.getY(DistanceUnit.CM),
                pose.getHeading(AngleUnit.RADIANS));
    }

    /**
     * <p>
     *      Converts an FTC SDK Pose2D to a Cyan Pose2D using a specified distance unit.
     * </p>
     *
     * @param pose The FTC SDK Pose2D object to convert.
     * @param unit The desired distance unit for the resulting Cyan Pose2D.
     * @return A Cyan Pose2D object with position in the specified unit and heading in radians.
     * @throws NullPointerException If pose or unit is null.
     */
    public static Pose2D toCyan(
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose,
            Distance.DistanceUnit unit) {
        return new Pose2D(
                pose.getX(RcToCyanDistanceUnit.toRC(unit)),
                pose.getY(RcToCyanDistanceUnit.toRC(unit)),
                pose.getHeading(AngleUnit.RADIANS));
    }
}
