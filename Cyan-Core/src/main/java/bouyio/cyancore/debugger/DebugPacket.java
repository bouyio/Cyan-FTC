package bouyio.cyancore.debugger;

import bouyio.cyancore.debugger.formating.StringIdentifiable;

/**
 * <p>Packets holding organised information that can be recorded using a Logger.<p/>
 * <p>Where {@link H} is the type of the header of the packet that has a string identifier attached.<p/>
 * <p>Where {@link V} is the type of the value of the packet.<p/>
 * @see Logger
 * @see StringIdentifiable
 * */
public class DebugPacket<H extends StringIdentifiable, V> {
    final H header;
    final V value;

    /**
     * <p>Creates a packet with specified header and value.<p/>
     * */
    public DebugPacket(H header, V value) {
        this.header = header;
        this.value = value;
    }

    /**<p>Returns the header of the packet.<p/>*/
    public H getHeader() {
        return header;
    }

    /**<p>Returns the value of the packet.<p/>*/
    public V getValue() {
        return value;
    }
}
