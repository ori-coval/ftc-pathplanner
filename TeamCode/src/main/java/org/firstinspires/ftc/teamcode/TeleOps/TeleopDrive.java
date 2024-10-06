package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.Scoring;
import org.firstinspires.ftc.teamcode.Commands.ClawSetState;
import org.firstinspires.ftc.teamcode.Commands.IntakeArmSetState;
import org.firstinspires.ftc.teamcode.Commands.IntakeByToggle;
import org.firstinspires.ftc.teamcode.Commands.LinearIntakeCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Commands.ResetFieldOrientedCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp
public class TeleopDrive extends MMTeleOp {

    MMRobot robot = MMRobot.getInstance();
    public TeleopDrive() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        MMRobot.getInstance().mmSystems.initDriveTrain();
        MMRobot.getInstance().mmSystems.initIntake();
        MMRobot.getInstance().mmSystems.initLinearIntake();
        MMRobot.getInstance().mmSystems.initElevator();
        MMRobot.getInstance().mmSystems.initIntakeArm();
        MMRobot.getInstance().mmSystems.initScoringArm();
        MMRobot.getInstance().mmSystems.initClaw();


//       Trigger leftTriggerCondition = new Trigger(()-> gamepad1.left_trigger > 0.05);
//       leftTriggerCondition.whenActive(
//               new LinearIntakeCommand(leftTriggerCondition)
//       );
//
//        leftTriggerCondition = new Trigger(()-> gamepad1.left_trigger > 0.05);
//        leftTriggerCondition.whenActive(
//                new IntakeArmSetState(MMRobot.getInstance().mmSystems.intakeArm, IntakeArm.Position.MID)
//        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new IntakeByToggle()
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ResetFieldOrientedCommand()
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ClawSetState(robot.mmSystems.claw, Claw.State.OPEN)
      );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new IntakeArmSetState(robot.mmSystems.intakeArm, IntakeArm.Position.MID)
        );
/*
     MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
             new MMPIDCommand(
                     MMRobot.getInstance().mmSystems.elevator,
                     73
             )
     );
*/
     MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
             new MMPIDCommand(
                     MMRobot.getInstance().mmSystems.elevator,
                     42
             )
     );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new MMPIDCommand(
                        MMRobot.getInstance().mmSystems.elevator,
                        1
                )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new Scoring(
                        MMRobot.getInstance().mmSystems.elevator,
                        MMRobot.getInstance().mmSystems.scoringArm,
                        MMRobot.getInstance().mmSystems.claw,
                        40

                )
        );

    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("yaw", MMRobot.getInstance().mmSystems.imu.getYawInDegrees());
        telemetry.update();

//        MMRobot.getInstance().mmSystems.claw.setPosition(0.2+gamepad1.left_trigger);
//        telemetry.addData("mero",MMRobot.getInstance().mmSystems.claw.getPosition());
//        telemetry.update();

//        MMRobot.getInstance().mmSystems.scoringArm.setPosition(gamepad1.left_trigger);
/*
        telemetry.addData("meow",MMRobot.getInstance().mmSystems.elevator.getHeight());
        telemetry.addData("Ticks",MMRobot.getInstance().mmSystems.elevator.motorLeftEncoder.getCounts());
        telemetry.update();

        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();

        MMRobot.getInstance().mmSystems.intake.setPower(gamepad1.right_trigger);

        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();
       FtcDashboard.getInstance().getTelemetry().addData("height",MMRobot.getInstance().mmSystems.elevator.getHeight());
       FtcDashboard.getInstance().getTelemetry().update();
*/


    }

}