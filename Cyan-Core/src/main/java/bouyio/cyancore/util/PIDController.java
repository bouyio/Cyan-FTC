package bouyio.cyancore.util;

/**
 * <p>A proportional, integral and derivative controller.<p/>
 * */
public class PIDController {

    private final PIDCoefficients coefficients;

    private double integralSum = 0;

    private double previousTime;
    private double previousError = 0;

    /**
     * <p>Creates a controller with the given coefficients.</p>
     * */
    public PIDController(PIDCoefficients coefficients) {
        this.coefficients = coefficients;

        previousTime = (double) System.currentTimeMillis() / 1000;
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
        double currentTime = (double) System.currentTimeMillis() / 1000;
        double deltaTime = currentTime - previousTime;
        double deltaError = error - previousError;

        integralSum += error * deltaTime;

        double result =
                error * coefficients.kP +
                integralSum * coefficients.kI +
                (deltaError / deltaTime) * coefficients.kD;

        previousTime = currentTime;
        previousError = error;
        return result;
    }

    /**
     * <p>Resets the integral sum of the controller.<p/>
     * */
    public void resetIntegralSum() {
        integralSum = 0;
    }
}
