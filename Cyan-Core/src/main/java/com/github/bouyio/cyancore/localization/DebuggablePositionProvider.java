package com.github.bouyio.cyancore.localization;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Exporter;
import com.github.bouyio.cyancore.debugger.Loggable;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.debugger.formating.Identifier;
import com.github.bouyio.cyancore.pathing.engine.PathFollower;

/**
 * <p>
 *      An implementation of the loggable interface tailored for position provider.
 *      Provides logger, exporter and, of course, position providing capabilities.
 * </p>
 * @see Loggable
 * @see PositionProvider
 * @see Logger
 * @see Exporter
 * */
public abstract class DebuggablePositionProvider implements PositionProvider, Loggable {

    // ----SYSTEM VERSION INFO---

    private final String SYSTEM_VERSION = "1.0";
    private final String SYSTEM_NAME = "DEBUGGABLE_POS_PROV";
    public String getSystemVersion() {return SYSTEM_VERSION;}
    public String getSystemName() {return SYSTEM_NAME;}

    // ----SYSTEM COMPONENTS---

    protected Logger logger;
    protected Exporter[] exporters;

    // ----SYSTEM METHODS---

    /**
     * <p>
     *    Attaches a logger to this instance of the DebuggablePositionProvider and all the encapsulated
     *    systems within it to record debug values.
     * <p/>
     *
     * */
    @Override
    public void attachLogger(Logger logger) {
        this.logger = logger;
    }

    /**
     * <p>
     *    Attaches an exporter to this instance of the DebuggablePositionProvider and all the encapsulated
     *    systems within it to dump debug values to external sources (ex. {@code Telemetry},
     *    {@code Standart Output}).
     * <p/>
     * */
    @Override
    public void attachExporters(Exporter... exporters) {
        this.exporters = exporters;
    }

    /**
     * <p>
     *    Exports all logged data of the DebuggablePositionProvider if logger and exporter are present.
     * <p/>
     *
     * @see DebuggablePositionProvider#attachExporters(Exporter...)
     * @see DebuggablePositionProvider#attachLogger(Logger)
     * */
    @Override
    public void log() {
        if (logger == null) return;

        logger.record(new DebugPacket<>(new Identifier(SYSTEM_NAME), SYSTEM_VERSION));
        debug();

        if (exporters == null) return;


        DebugPacket[] logDump = logger.dump();

        for (Exporter e : exporters) {
            e.accept(logDump);
            e.export();
        }
    }

    /**
     * <p>Should write all the debug information of the system to the logger (if present).</p>
     * */
    abstract void debug();
}
