package bouyio.cyancore.debugger;

public class Debuggers {

    private static Logger globalLogger;

    public static void init() {
        globalLogger = new Logger();
    }

    public static Logger getGlobalLogger() {
        return globalLogger;
    }
}
