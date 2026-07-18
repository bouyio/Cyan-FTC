package com.github.bouyio.cyancore.debugger;

/**
 * An interface that allows logger to output to any external source.
 * @see Logger
 * @see Loggable
 */
public interface Exporter {

    /**
     * <p>
     *      Gets dump data from the logger and writes it to local buffer.
     * </p>
     * @see Logger#dump()
     * */
    void accept(DebugPacket[] dump);

    /**
     * <p>
     *     Prints and clears buffer.
     * </p>
     * */
    void export();
}
