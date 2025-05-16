package com.github.bouyio.cyancore.util;

/**
 * <p>Stores tunable coefficients required for a PID controller.<p/>
 * @see PIDCoefficients
 * */
public class PIDCoefficients {
    public PIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDCoefficients() {}

    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
}
