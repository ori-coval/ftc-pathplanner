package org.firstinspires.ftc.teamcode.MMLib.Utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * this class represents ur robot battery
 */
public class MMBattery {

    private final VoltageSensor battery;

    public MMBattery(HardwareMap hardwareMap) {
        this.battery = hardwareMap.voltageSensor.iterator().next();
    }

    public double getVoltage() {
        return battery.getVoltage();
    }

}
