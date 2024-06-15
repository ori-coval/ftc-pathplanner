package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;

@TeleOp
public class Test extends MMTeleOp {

    VoltageSensor sensor;

    public Test() {
        super(false);
    }

    @Override
    public void main() {
        sensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("Voltage", sensor.getVoltage());
        telemetry.update();

    }
}
