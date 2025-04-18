package bouyio.cyancore.util;

public class StuckOscillationController {
    private double prevTime = 0;
    private double prevAbsError = 0;

    public double update(double error) {

        double currentTime = System.currentTimeMillis();

        if (prevAbsError == 0) {
            prevAbsError = Math.abs(error);
            prevTime = System.currentTimeMillis();
            return 0;
        }

        double deltaTime = (currentTime - prevTime) / 100;
        double rateChangeDeOscillator = 0;

        if (Math.abs(error) > prevAbsError) {
            rateChangeDeOscillator = error / deltaTime;
        }

        prevAbsError = Math.abs(error);

        return rateChangeDeOscillator;
    }
}
