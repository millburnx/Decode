//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.telemetry.SelectScope;
import com.pedropathing.telemetry.Selector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public abstract class SelectableOpModePanels extends OpMode {
    private final Selector<Supplier<OpMode>> selector;
    private OpMode selectedOpMode;
    private static final String[] MESSAGE = new String[]{"Use the d-pad to move the cursor.", "Press right bumper to select.", "Press left bumper to go back."};

    public SelectableOpModePanels(String name, Consumer<SelectScope<Supplier<OpMode>>> opModes) {
        this.selector = Selector.create(name, opModes, MESSAGE);
        this.selector.onSelect((opModeSupplier) -> {
            this.onSelect();
            this.selectedOpMode = (OpMode)opModeSupplier.get();
            this.selectedOpMode.gamepad1 = this.gamepad1;
            this.selectedOpMode.gamepad2 = this.gamepad2;
            this.selectedOpMode.telemetry = this.telemetry;
            this.selectedOpMode.hardwareMap = this.hardwareMap;
            this.selectedOpMode.init();
        });
    }

    protected void onSelect() {
    }

    protected void onLog(List<String> line) {
    }

    public final void init() {
    }

    GamepadManager gm1 = PanelsGamepad.INSTANCE.getFirstManager();
    GamepadManager gm2 = PanelsGamepad.INSTANCE.getSecondManager();

    Gamepad gp1 = gm1.asCombinedFTCGamepad(gamepad1);
    Gamepad gp2 = gm2.asCombinedFTCGamepad(gamepad2);

    TelemetryManager tel = PanelsTelemetry.INSTANCE.getTelemetry();

    public final void init_loop() {
        if (this.selectedOpMode == null) {
            if (!this.gp1.dpadUpWasPressed() && !this.gp2.dpadUpWasPressed()) {
                if (!this.gp1.dpadDownWasPressed() && !this.gp2.dpadDownWasPressed()) {
                    if (!this.gp1.rightBumperWasPressed() && !this.gp2.rightBumperWasPressed()) {
                        if (this.gp1.leftBumperWasPressed() || this.gp2.leftBumperWasPressed()) {
                            this.selector.goBack();
                        }
                    } else {
                        this.selector.select();
                    }
                } else {
                    this.selector.incrementSelected();
                }
            } else {
                this.selector.decrementSelected();
            }

            List<String> lines = this.selector.getLines();

            for(String line : lines) {
                this.tel.addLine(line);
            }

            this.onLog(lines);
        } else {
            this.selectedOpMode.init_loop();
        }

    }

    public final void start() {
        if (this.selectedOpMode == null) {
            throw new RuntimeException("No OpMode selected!");
        } else {
            this.selectedOpMode.start();
        }
    }

    public final void loop() {
        this.selectedOpMode.loop();
    }

    public final void stop() {
        if (this.selectedOpMode != null) {
            this.selectedOpMode.stop();
        }

    }
}
