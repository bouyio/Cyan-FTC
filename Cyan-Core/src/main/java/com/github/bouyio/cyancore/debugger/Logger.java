package com.github.bouyio.cyancore.debugger;

import java.util.LinkedList;
import java.util.List;

import com.github.bouyio.cyancore.debugger.formating.Identifier;
import com.github.bouyio.cyancore.debugger.formating.MessageLevel;

/** A logger system that records packets of data critical for debugging.*/
public class Logger {

    private final List<DebugPacket> buffer;

    private boolean isBufferFull = false;

    private final int BUFFER_SIZE;
    private int index = 0;

    /** Creates a Logger instance with the default data buffer size of 50 debug packets.*/
    public Logger() {
        this(50);
    }

    /** Creates a Logger instance with a custom data buffer size. */
    public Logger(int size) {
        BUFFER_SIZE = size;
        buffer = new LinkedList<>();
    }

    /**
     * <p>
     *     Stores the sent debug packet to the buffer until dumped or cleared.
     *     It can be access by dumping the buffer.
     *     If the buffer is almost full it will record a warning stating that the buffer is full and needs to be dumped or cleared.
     *     If the buffer is full no packet will be recorded.
     * <p/>
     *
     * @param packet The packet to be recorded.
     * */
    public void record(DebugPacket packet) {

        if (isBufferFull) return;

        buffer.add(packet);
        index++;

        if (index == BUFFER_SIZE - 2) {
            logMessage(MessageLevel.WARNING,
                    "Logger buffer has reached its maximum Capacity and will NOT be able to record until dumped or cleared.");
            isBufferFull = true;
        }

    }

    /**
     *
     * <p>
     *     Records a debug packet with a specified header and value to the buffer which can be accessed by dumping the buffer.
     *     If the buffer is full the packet will NOT be recorded.
     * <p/>
     *
     * @param header The header of the packet.
     * @param value The value of the packet.
     * */
    public void logValue(String header, Object value) {
        Identifier i = new Identifier(header);
        DebugPacket<Identifier, Object> packet = new DebugPacket<>(i, value);

        record(packet);
    }

    /**
     *
     * <p>
     *     Records a message with a specified {@link MessageLevel} to the buffer which can be accessed by dumping the buffer.
     *     If the buffer is full the message will NOT be recorded.
     * <p/>
     *
     * @param msgLvl The message type that will be displayed as the header of the packet.
     * @param content The content of the message that will be displayed as the value of the packet.
     * */
    public void logMessage(MessageLevel msgLvl, String content) {
        DebugPacket<MessageLevel, String> packet = new DebugPacket<>(msgLvl, content);
        record(packet);
    }

    /**
     *
     * <p>
     *     Records a {@link MessageLevel#DEBUG} message to the buffer which can be accessed by dumping the buffer.
     *     If the buffer is full the message will NOT be recorded.
     * <p/>
     *
     * @param content The content of the message that will be displayed as the value of the packet.
     * */
    public void logMessage(String content) {
        logMessage(MessageLevel.DEBUG, content);
    }

    /**
     *
     * <p>
     *      Clears the buffer and resets the its full status.
     * <p/>
     *
     * */
    public void clearBuffer() {
        buffer.clear();
        index = 0;
        isBufferFull = false;
    }

    /**
     *
     * <p>
     *      Clears the buffer and returns all of its contents since its last clearance.
     * <p/>
     *
     * @return All of the contents of the buffer in the order they have been added.
     * */
    public DebugPacket[] dump() {
        DebugPacket[] copy = new DebugPacket[BUFFER_SIZE];
        buffer.toArray(copy);

        clearBuffer();
        return copy;
    }
}
