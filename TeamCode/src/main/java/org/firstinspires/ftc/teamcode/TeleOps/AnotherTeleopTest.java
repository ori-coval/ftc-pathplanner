package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMInitMethods;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp
public class AnotherTeleopTest extends MMTeleOp {
    MMRobot robot = MMRobot.getInstance();
    public AnotherTeleopTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }
    @Override
    public void onInit() {
        MMInitMethods.initLinearIntake();
    }

    @Override
    public void run() {
        super.run();
        robot.mmSystems.linearIntake.setPosition(gamepad1.left_trigger);
    }
}
