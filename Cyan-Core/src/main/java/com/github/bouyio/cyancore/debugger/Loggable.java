package com.github.bouyio.cyancore.debugger;

/**
 * <p>
 *     This interface streamlines the debugging process by providing a standardized set of methods
 *     that any class can implement.
 * </p>
 *
 * <p>
 *      Supported debugging functions include logging values and exporting them to an external source.
 * </p>
 *
 * @see Logger
 * @see Exporter
 * */
public interface Loggable {

    /**
     * <p>Attaches a logger to the instance.</p>
     * @param logger The logger to be attached.
     * */
    void attachLogger(Logger logger);

    /**
     * <p>Attaches exporters to the instance.</p>
     * @param exporters The exporters to be attached.
     * */
    void attachExporters(Exporter... exporters);

    /**
     * <p>
     *     Should log all debug variables - Dump the logger - Provide the exporter with the dumps of
     *     the logger - Clear and display the exporter's buffer.
     * </p>
     *
     * <p>Implementation example:</p>
     * <pre>
     *     {@code
     *     // Log every debug value
     *     logger.log("header", value);
     *
     *     // Empty the logger
     *     DebugPacket[] logDump = logger.dump();
     *
     *     // Provding the exported with the logger's dump
     *     // And printing the exporter's buffer
     *     for (Exporter exporter : exporters) {
     *         exporter.accept(logDump);
     *         exporter.export();
     *     }
     *     }
     * </pre>
     * @implNote Null checks not included
     * @see Logger
     * @see Exporter
     * */
    void log();
}
