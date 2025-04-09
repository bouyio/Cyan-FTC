package bouyio.cyancore.util;

public class PIDController {
    private final double kP;
    private final double kD;
    private final double kI;

    private double integralSum = 0;

    private double previousTime;
    private double previousError = 0;

    public PIDController(PIDCoefficients coefficients) {
        this(coefficients.kP, coefficients.kD, coefficients.kI);
    }

    public PIDController(double proportionalCoefficient,
                         double derivativeCoefficient,
                         double integralCoefficient) {
        kP = proportionalCoefficient;
        kD = derivativeCoefficient;
        kI = integralCoefficient;

        previousTime = (double) System.currentTimeMillis() / 1000;
    }

    public double update(double error) {
        double currentTime = (double) System.currentTimeMillis() / 1000;
        double deltaTime = currentTime - previousTime;
        double deltaError = error - previousError;

        integralSum += error * deltaTime;

        double result = error * kP + integralSum * kI + (deltaError / deltaTime) * kD;

        previousTime = currentTime;
        previousError = error;
        return result;
    }

    public void resetIntegralSum() {
        integralSum = 0;
    }
}
