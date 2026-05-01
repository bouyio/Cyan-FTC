package com.github.bouyio.cyancore.localization;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Exporter;
import com.github.bouyio.cyancore.debugger.Loggable;
import com.github.bouyio.cyancore.debugger.Logger;

public abstract class DebuggablePositionProvider implements PositionProvider, Loggable {
    protected Logger logger;
    protected Exporter[] exporters;

    @Override
    public void attachLogger(Logger logger) {
        this.logger = logger;
    }

    @Override
    public void attachExporters(Exporter... exporters) {
        this.exporters = exporters;
    }

    @Override
    public void log() {
        if (logger == null) return;

        debug();

        if (exporters == null) return;


        DebugPacket[] logDump = logger.dump();

        for (Exporter e : exporters) {
            e.accept(logDump);
            e.export();
        }
    }

    abstract void debug();
}
