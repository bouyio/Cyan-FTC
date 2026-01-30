package com.github.bouyio.cyanftc.util;

import com.github.bouyio.cyancore.geomery.Pose2D;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RcToCyanPose {
    public static Pose2D toCyan(org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose) {
        return new Pose2D(
                pose.getX(DistanceUnit.CM),
                pose.getY(DistanceUnit.CM),
                pose.getHeading(AngleUnit.RADIANS));
    }
}
