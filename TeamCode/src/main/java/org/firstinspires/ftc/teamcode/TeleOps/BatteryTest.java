package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MMLib.Utils.MMBattery;
import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;

@TeleOp
public class BatteryTest extends MMTeleOp {

    MMBattery mmBattery;

    public BatteryTest() {
        super(false);
    }

    @Override
    public void main() {
        mmBattery = new MMBattery(hardwareMap);
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("Voltage", mmBattery.getVoltage());
        telemetry.update();

    }
}
