package bouyio.cyancore.debugger;

import java.util.ArrayList;
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
        buffer = new ArrayList<>(BUFFER_SIZE);
    }

    public void record(DebugPacket packet) {

        if (hasBufferOverflowed) return;

        if (index == buffer.size() - 2) {
            hasBufferOverflowed = true;
            logMessage(MessageLevel.WARNING,
                    "Logger buffer has reached its maximum Capacity and will NOT be able to record until dumped");
        }

        buffer.set(index, packet);
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
        DebugPacket[] buffer = new DebugPacket[]{};
        this.buffer.toArray(buffer);

        index = 0;
        hasBufferOverflowed = false;

        this.buffer.clear();
        return buffer;
    }
}
