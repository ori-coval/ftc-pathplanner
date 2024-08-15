package org.firstinspires.ftc.teamcode.MMLib.Examples.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MMLib.Utils.MMBattery;
import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp
public class BatteryTest extends MMTeleOp {

    MMBattery mmBattery;

    public BatteryTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
        mmBattery = new MMBattery(hardwareMap);
    }

    @Override
    public void run() {
        telemetry.addData("Voltage", mmBattery.getVoltage());
        telemetry.update();
    }
}
