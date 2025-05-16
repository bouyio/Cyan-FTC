package com.github.bouyio.cyancore.debugger.formating;

/**
 * <p>
 *    Allows for use of any object as a header for a packet.
 * <p/>
 *
 * @see com.github.bouyio.cyancore.debugger.DebugPacket
 * */
public interface StringIdentifiable {

    /**<p>Returns the identifier.<p/>*/
    String getIdentifier();
}
