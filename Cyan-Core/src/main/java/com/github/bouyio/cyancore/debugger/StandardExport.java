package com.github.bouyio.cyancore.debugger;

import com.github.bouyio.cyancore.debugger.formating.MessageLevel;

public class StandardExport implements Exporter {

    private final int DUMP_SIZE;
    private final int MAXIMUM_DUMPS;

    private DebugPacket[][] buffer;
    private int bufferPointer = 0;


    public StandardExport(int dumpSize, int maximumDumps) {
        DUMP_SIZE = dumpSize;
        MAXIMUM_DUMPS = maximumDumps;

        buffer = new DebugPacket[maximumDumps + 1][dumpSize];
    }

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

    @Override
    public void export() {
        for (DebugPacket[] dump : buffer) {
            if (dump == null) break;
            for (DebugPacket packet : dump) {
                 if (packet == null) continue;
                System.out.printf("%s: %s", packet.header.getIdentifier(), packet.getValue());
            }
        }

        buffer = new DebugPacket[MAXIMUM_DUMPS + 1][DUMP_SIZE];
        bufferPointer = 0;

    }
}
