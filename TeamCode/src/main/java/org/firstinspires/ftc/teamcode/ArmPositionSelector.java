package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class ArmPositionSelector {
    private Telemetry telemetry;
    private int selectedDistanceID;
    private int selectedHeightID;
    private int getSelectedSideID;

    public ArmPositionSelector(Telemetry telemetry) {
        this.selectedDistanceID = 0;
        this.selectedHeightID = 0;
        this.getSelectedSideID = 0;
    }

    public void moveSelectedDistanceIdRight() {
        if (!(selectedDistanceID == 1)) {selectedDistanceID++;
            }
    }
    public void moveSelectedDistanceIdLeft() {
        if (!(selectedDistanceID == 0)) {
            selectedDistanceID--;
        }
    }
    public void moveSelectedHeightIdUp() {
        if (!(selectedDistanceID == 2)) {
            selectedDistanceID++;
        }
    }
    public void moveSelectedHeightIdDown() {
        if (!(selectedDistanceID == 0)) {
            selectedDistanceID--;
        }
    }

    public void moveSelectedSideIdRight() {
        if (!(selectedDistanceID == 2)) {
            selectedDistanceID++;
        }
    }

    public void moveSelectedSideIdLeft() {
        if (!(selectedDistanceID == 0)) {
            selectedDistanceID--;
        }
    }


}

