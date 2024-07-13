package org.firstinspires.ftc.teamcode.MMLib.OpModes;

import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;

public class MMBatteryTest extends MMTeleOp {

    public MMBatteryTest() {
        super(false);
    }

    @Override
    public void main() {}

    @Override
    public void run() {
        telemetry.addData(
                "Battery",
                MMRobot.getInstance().mmSystems.battery.getVoltage()
        );
        super.run();
    }
}
