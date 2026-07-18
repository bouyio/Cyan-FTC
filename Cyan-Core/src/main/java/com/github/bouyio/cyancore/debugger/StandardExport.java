package com.github.bouyio.cyancore.debugger;

import com.github.bouyio.cyancore.debugger.formating.MessageLevel;

/**
 * An export system that allows logger to output to the standard output.
 *
 * @see Logger
 * @see Exporter
 * @see System#out
 */
public class StandardExport implements Exporter {

    private final int DUMP_SIZE;
    private final int MAXIMUM_DUMPS;

    private DebugPacket[][] buffer;
    private int bufferPointer = 0;

    /**
     * Creates a StandardExporter instance with specified dump buffer size and maximum amount of dumps.
     * @param dumpSize The maximum amount of entries each dump can store.
     * @param maximumDumps The maximum amount of dumps the buffer can store before cleared.
     * */
    public StandardExport(int dumpSize, int maximumDumps) {
        DUMP_SIZE = dumpSize;
        MAXIMUM_DUMPS = maximumDumps;

        buffer = new DebugPacket[maximumDumps + 1][dumpSize];
    }

    /**
     * <p>
     *      Gets dump data from the logger and writes it to local buffer.
     * </p>
     *
     * <p>
     *     If the maximum amount of dumps is exceeded a warning stating that the buffer is full and
     *     needs to be dumped or cleared is recorded.
     * </p>
     *
     * <p>
     *     If the size of a dump exceeds the maximum size the dump is not recorded instead an error
     *     notifying the user that the exceeds the set size limit is recorded.
     * </p>
     *
     * @apiNote To clear the buffer call {@link StandardExport#export()}.
     *
     * @see Logger#dump()
     * */
    @Override
    public void accept(DebugPacket[] dump) {
        if (bufferPointer == MAXIMUM_DUMPS) {
            buffer[bufferPointer][0] = new DebugPacket<>(MessageLevel.WARNING,
                    "The buffer of exporter:  " + hashCode() + " is FULL!. " +
                            "Writing to exporter will be disabled until emptied.");

            bufferPointer++;
            return;
        }

        if (bufferPointer > MAXIMUM_DUMPS)
            return;

        if (dump.length > DUMP_SIZE) {
            buffer[bufferPointer][0] = new DebugPacket<>(MessageLevel.ERROR,
                    "Dump exceeds the defined dump size limit of " + DUMP_SIZE + ".");

            bufferPointer++;
            return;
        }

        buffer[bufferPointer] = dump;
        bufferPointer++;

    }

    /**
     * <p>
     *     Clears the buffer and prints its contents to the standard output.
     * </p>
     * @see System#out
     * @see java.io.PrintStream#printf(String, Object...)
     * */
    @Override
    public void export() {
        for (DebugPacket[] dump : buffer) {
            if (dump == null) break;
            for (DebugPacket packet : dump) {
                if (packet == null) continue;
                System.out.printf("%s: %s\n", packet.header.getIdentifier(), packet.getValue());
            }
        }

        buffer = new DebugPacket[MAXIMUM_DUMPS + 1][DUMP_SIZE];
        bufferPointer = 0;

    }
}
