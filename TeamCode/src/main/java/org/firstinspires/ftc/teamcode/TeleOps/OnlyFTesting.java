package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.Intake;
import org.firstinspires.ftc.teamcode.Commands.ClawSetState;
import org.firstinspires.ftc.teamcode.Commands.LinearIntakeCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;
@TeleOp
public class OnlyFTesting extends MMTeleOp {
        int sv = 0;
    public OnlyFTesting() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initLinearIntake();
        MMRobot.getInstance().mmSystems.initScoringArm();
        MMRobot.getInstance().mmSystems.initClaw();
        Trigger leftTriggerCondition = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getRightX() > 0.05
        );

        leftTriggerCondition.whenActive(
                new LinearIntakeCommand(leftTriggerCondition, ()->MMRobot.getInstance().mmSystems.gamepadEx1.getRightX())
        );
    }

    @Override
    public void run() {
        super.run();

        if(sv==0){
            sv++;
            new LinearIntakeCommand(new Trigger(()->false), ()->gamepad1.right_stick_x).schedule();
        }


//        MMRobot.getInstance().mmSystems.linearIntake.setPosition(gamepad1.left_trigger);

//        MMRobot.getInstance().mmSystems.scoringArm.setPosition(gamepad1.left_trigger);
//        MMRobot.getInstance().mmSystems.claw.setPosition(gamepad1.right_trigger);
//        telemetry.addData("Scoring arm: " ,MMRobot.getInstance().mmSystems.scoringArm.getPosition());
//        telemetry.addData("j",MMRobot.getInstance().mmSystems.claw.getPosition());
//        telemetry.update();

    }
}
