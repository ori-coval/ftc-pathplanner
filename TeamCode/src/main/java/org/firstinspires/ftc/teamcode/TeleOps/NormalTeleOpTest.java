package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp
public class NormalTeleOpTest extends MMTeleOp {

    public NormalTeleOpTest(){
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    CuttleMotor motor;

    @Override
    public void onInit() {


    }

    @Override
    public void run() {
        super.run();

    }
}