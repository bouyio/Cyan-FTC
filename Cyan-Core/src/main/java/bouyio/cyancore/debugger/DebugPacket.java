package bouyio.cyancore.debugger;

import bouyio.cyancore.debugger.formating.StringIdentifiable;

public class DebugPacket<H extends StringIdentifiable, V> {
    final H header;
    final V value;

    public DebugPacket(H header, V value) {
        this.header = header;
        this.value = value;
    }

    public H getHeader() {
        return header;
    }

    public V getValue() {
        return value;
    }
}
