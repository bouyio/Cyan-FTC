package bouyio.cyancore.debugger;

import java.util.LinkedList;
import java.util.List;

import bouyio.cyancore.debugger.formating.Identifier;
import bouyio.cyancore.debugger.formating.MessageLevel;

public class Logger {

    private List<DebugPacket> buffer;

    private boolean hasBufferOverflowed = false;

    private final int BUFFER_SIZE;
    private int index = 0;

    public Logger() {
        this(50);
    }

    public Logger(int size) {
        BUFFER_SIZE = size;
        buffer = new LinkedList<>();
    }

    public void record(DebugPacket packet) {

        if (hasBufferOverflowed) return;

        if (index == BUFFER_SIZE - 2) {
            hasBufferOverflowed = true;
            logMessage(MessageLevel.WARNING,
                    "Logger buffer has reached its maximum Capacity and will NOT be able to record until dumped");
        }

        buffer.add(packet);
        index++;
    }

    public void logValue(String header, Object value) {
        Identifier i = new Identifier(header);
        DebugPacket<Identifier, Object> packet = new DebugPacket<>(i, value);

        record(packet);
    }

    public void logMessage(MessageLevel msgLvl, String content) {
        DebugPacket<MessageLevel, String> packet = new DebugPacket<>(msgLvl, content);
        record(packet);
    }

    public void logMessage(String content) {
        logMessage(MessageLevel.DEBUG, content);
    }

    public void clearBuffer() {
        buffer.clear();
    }

    public DebugPacket[] dump() {
        DebugPacket[] copy = new DebugPacket[BUFFER_SIZE];
        buffer.toArray(copy);
        index = 0;
        hasBufferOverflowed = false;

        clearBuffer();
        return copy;
    }
}
