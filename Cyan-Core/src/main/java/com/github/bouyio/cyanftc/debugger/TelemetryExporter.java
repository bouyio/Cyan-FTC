package com.github.bouyio.cyanftc.debugger;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Exporter;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.debugger.StandardExport;
import com.github.bouyio.cyancore.debugger.formating.MessageLevel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * An export system that allows logger to output to the driver station telemetry.
 *
 * @see Logger
 * @see Exporter
 * @see Telemetry
 */
public class TelemetryExporter implements Exporter {

    List<List<DebugPacket>> buffer;

    private final int MAXIMUM_DUMPS;
    private boolean isBufferFull = false;

    private final Telemetry telemetry;

    /**
     * Creates a Telemetry instance with specified telemetry output and specified maximum amount of dumps.
     * @param telemetry The telemetry that the exported outputs to.
     * @param maximumDumps The maximum amount of dumps the buffer can store before cleared.
     * */
    public TelemetryExporter(Telemetry telemetry, int maximumDumps) {
        MAXIMUM_DUMPS = maximumDumps;
        this.telemetry = telemetry;

        buffer = new ArrayList<>(maximumDumps);
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
     * @implNote This exporter uses a dynamic buffer for storing; any size dump that can fit inside the JVM's memory is acceptable.
     *
     * @apiNote To clear the buffer call {@link StandardExport#export()}.
     *
     * @see Logger#dump()
     * */
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

    /**
     * <p>
     *     Clears the buffer and prints its contents to the telemetry.
     * </p>
     * @see Telemetry#addData(String, Object)
     * @apiNote To see the data in telemetry call {@link TelemetryExporter#update()} .
     * */
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

    /**
     * <p>Updates the telemetry.</p>
     * @apiNote Please call <strong>only once</strong> per cycle.
     * @see Telemetry#update()
     * */
    public void update() {
        telemetry.update();
    }
}
