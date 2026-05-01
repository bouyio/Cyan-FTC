package com.github.bouyio.cyancore.debugger;

public interface Exporter {
    void accept(DebugPacket[] dump);
    void export();
}
