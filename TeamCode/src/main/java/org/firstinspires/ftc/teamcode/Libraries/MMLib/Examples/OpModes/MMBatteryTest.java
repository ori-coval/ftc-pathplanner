package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.OpModes;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

public class MMBatteryTest extends MMTeleOp {

    public MMBatteryTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {}

    @Override
    public void run() {
        telemetry.addData(
                "Battery",
                MMRobot.getInstance().mmSystems.battery.getVoltage()
        );
        telemetry.update();
    }
}
