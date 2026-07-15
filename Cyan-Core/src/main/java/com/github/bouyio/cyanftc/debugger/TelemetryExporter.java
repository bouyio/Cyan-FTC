package com.github.bouyio.cyanftc.debugger;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Exporter;
import com.github.bouyio.cyancore.debugger.formating.MessageLevel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TelemetryExporter implements Exporter {

    List<List<DebugPacket>> buffer;

    private final int MAXIMUM_DUMPS;
    private boolean isBufferFull = false;

    private final Telemetry telemetry;

    public TelemetryExporter(Telemetry telemetry, int maximumDumps) {
        MAXIMUM_DUMPS = maximumDumps;
        this.telemetry = telemetry;

        buffer = new ArrayList<>(maximumDumps);
    }

    @Override
    public void accept(DebugPacket[] dump) {
        if (isBufferFull) return;

        if (buffer.size() == MAXIMUM_DUMPS) {
            List<DebugPacket> temp = new ArrayList<>();

            temp.add(new DebugPacket<>(MessageLevel.WARNING,
                    "The buffer of exporter:  " + hashCode() + " is FULL!. " +
                            "Writing to exporter will be disabled until emptied."));

            buffer.add(temp);
            isBufferFull = true;
            return;
        }
        buffer.add(Arrays.asList(dump));
    }

    @Override
    public void export() {
        for (List<DebugPacket> dump : buffer) {
            if (dump == null) break;
            for (DebugPacket packet : dump) {
                if (packet == null) continue;
                telemetry.addData(packet.getHeader().getIdentifier(), packet.getValue());
            }
        }

        buffer.clear();
        isBufferFull = false;
    }

    public void update() {
        telemetry.update();
    }
}
