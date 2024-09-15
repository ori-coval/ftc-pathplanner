package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMInitMethods;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp
public class NormalTeleOpTest extends MMTeleOp {
    MMRobot robot = MMRobot.getInstance();


    public NormalTeleOpTest(){
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
        MMInitMethods.initArmAngle();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.armAngle.setPosition(gamepad1.left_trigger);
        telemetry.addData("psition",MMRobot.getInstance().mmSystems.armAngle.getPosition());
        telemetry.update();
    }
}