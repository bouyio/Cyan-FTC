package com.github.bouyio.cyancore.debugger;

public interface Loggable {
    void attachLogger(Logger logger);

    void log();
}
