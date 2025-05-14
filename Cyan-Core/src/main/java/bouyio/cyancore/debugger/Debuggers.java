package bouyio.cyancore.debugger;

/**
 * <p>Contains globally accessible default instances of the all debugging tools.<p/>
 * @see Logger
 * */
public class Debuggers {

    private static Logger globalLogger = null;

    /**<p>Instantiates and configures the debugging tools.<p/>*/
    public static void init() {
        globalLogger = new Logger();
    }

    /**@return The default logger.*/
    public static Logger getGlobalLogger() {
        if (globalLogger == null) init();
        return globalLogger;
    }
}
