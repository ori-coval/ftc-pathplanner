package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ClawSetState;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;
@TeleOp
public class OnlyFTesting extends MMTeleOp {

    public OnlyFTesting() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initScoringArm();
        MMRobot.getInstance().mmSystems.initClaw();

    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.scoringArm.setPosition(gamepad1.left_trigger);
        MMRobot.getInstance().mmSystems.claw.setPosition(gamepad1.right_trigger);
        telemetry.addData("Scoring arm: " ,MMRobot.getInstance().mmSystems.scoringArm.getPosition());
        telemetry.addData("j",MMRobot.getInstance().mmSystems.claw.getPosition());
        telemetry.update();

    }
}
