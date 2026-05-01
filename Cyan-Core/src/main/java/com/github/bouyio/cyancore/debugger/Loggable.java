package com.github.bouyio.cyancore.debugger;

public interface Loggable {
    void attachLogger(Logger logger);

    void attachExporters(Exporter... exporters);

    void log();
}
