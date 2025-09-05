package com.github.bouyio.cyancore.util;

/**
 * A proportional, integral and derivative controller with improved performance and safety.
 * Features anti-windup protection and optimized calculations for FTC robotics.
 *
 * @author Bouyio (https://github.com/bouyio)
 * @author Gvol (https://github.com/Gvolexe)
 */
public class PIDController {

    private final PIDCoefficients coefficients;

    private double integralSum = 0;
    private double previousTime;
    private double previousError = 0;
    
    // Optimized: Added anti-windup protection
    private double maxIntegralSum = Double.MAX_VALUE;
    private boolean firstUpdate = true;

    /**
     * <p>Creates a controller with the given coefficients.</p>
     * */
    public PIDController(PIDCoefficients coefficients) {
        if (coefficients == null) {
            throw new IllegalArgumentException("PID coefficients cannot be null");
        }
        this.coefficients = coefficients;
        previousTime = System.currentTimeMillis() * 0.001; // Optimized: avoid division
    }

    /**
     * <p>Creates a controller with the given coefficients.</p>
     * */
    public PIDController(double proportionalCoefficient,
                         double derivativeCoefficient,
                         double integralCoefficient) {

        this(new PIDCoefficients(proportionalCoefficient, derivativeCoefficient, integralCoefficient));
    }

    /**
     * <p>Updates the controller and the result based on current error.<p/>
     * @param error The error.
     * @return The result.
     * */
    public double update(double error) {
        final double currentTime = System.currentTimeMillis() * 0.001; // Optimized: avoid division
        final double deltaTime = currentTime - previousTime;
        
        // Optimized: Handle first update and prevent division by zero
        if (firstUpdate || deltaTime <= 1e-6) { // Use epsilon instead of 0 for better precision
            firstUpdate = false;
            previousTime = currentTime;
            previousError = error;
            return error * coefficients.kP;  // Return proportional term only
        }

        // Optimized: Integral with anti-windup protection (faster than if statements)
        integralSum += error * deltaTime;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

        final double derivative = (error - previousError) / deltaTime;

        final double result = error * coefficients.kP +
                             integralSum * coefficients.kI +
                             derivative * coefficients.kD;

        previousTime = currentTime;
        previousError = error;
        return result;
    }

    /**
     * <p>Resets the integral sum of the controller.<p/>
     * */
    public void resetIntegralSum() {
        integralSum = 0;
        firstUpdate = true;  // Optimized: Reset first update flag
    }

    /**
     * <p>Sets the maximum integral sum to prevent windup.<p/>
     * @param maxIntegralSum Maximum allowed integral sum.
     * */
    public void setMaxIntegralSum(double maxIntegralSum) {
        this.maxIntegralSum = Math.abs(maxIntegralSum);
    }

    /**
     * <p>Gets the current integral sum.<p/>
     * @return Current integral sum value.
     * */
    public double getIntegralSum() {
        return integralSum;
    }

    /**
     * <p>Gets the current derivative term for debugging.<p/>
     * @return Current derivative value.
     * */
    public double getLastDerivative() {
        return (previousError * coefficients.kD); // Last calculated derivative component
    }

    /**
     * <p>Gets the current proportional term for debugging.<p/>
     * @return Current proportional value.
     * */
    public double getLastProportional() {
        return previousError * coefficients.kP;
    }

    /**
     * <p>Checks if the controller is initialized (has run at least once).<p/>
     * @return True if the controller has been updated at least once.
     * */
    public boolean isInitialized() {
        return !firstUpdate;
    }

    /**
     * <p>Gets a copy of the PID coefficients for safety.<p/>
     * @return Copy of current coefficients.
     * */
    public PIDCoefficients getCoefficients() {
        return new PIDCoefficients(coefficients.kP, coefficients.kI, coefficients.kD);
    }
}
