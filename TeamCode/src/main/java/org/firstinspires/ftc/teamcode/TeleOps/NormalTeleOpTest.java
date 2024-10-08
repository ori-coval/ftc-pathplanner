package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp
public class NormalTeleOpTest extends MMTeleOp {

    public NormalTeleOpTest(){
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    CuttleMotor motor;

    @Override
    public void onInit() {

//        CRServo s1 = hardwareMap.crservo.get("s1");
//        CRServo s2 = hardwareMap.crservo.get("s2");
//        CRServo s3 = hardwareMap.crservo.get("s3");
//        CRServo s4 = hardwareMap.crservo.get("s4");
//        CRServo s5 = hardwareMap.crservo.get("s5");

    }

    @Override
    public void run() {
        super.run();

    }
}