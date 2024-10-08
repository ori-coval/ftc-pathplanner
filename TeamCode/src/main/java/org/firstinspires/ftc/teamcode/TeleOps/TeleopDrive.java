package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeArmDownAndStartRoller;
import org.firstinspires.ftc.teamcode.Commands.IntakeArmSetState;
import org.firstinspires.ftc.teamcode.Commands.IntakeByToggle;
import org.firstinspires.ftc.teamcode.Commands.LinearIntakeCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
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
//        MMRobot.getInstance().mmSystems.initElevator();
        MMRobot.getInstance().mmSystems.initIntakeArm();
//        MMRobot.getInstance().mmSystems.initScoringArm();

        addRunnableOnInit(
                () -> MMRobot.getInstance().mmSystems.linearIntake.setPosition(0)
        );

        addCommandsOnRun(
             new IntakeArmSetState(IntakeArm.Position.IN)
        );


        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).toggleWhenActive(
                new IntakeArmDownAndStartRoller()
        );

//
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
//                new ResetFieldOrientedCommand()
//        );
//
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new ClawSetState(robot.mmSystems.claw, Claw.State.OPEN)
//      );
        Trigger leftTriggerCondition = new Trigger(() -> gamepad1.left_trigger > 0.05);
        leftTriggerCondition.whenActive(
                new SequentialCommandGroup(
                        new IntakeArmSetState(IntakeArm.Position.MID),
                        new LinearIntakeCommand(leftTriggerCondition)
                )
        );

//     MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//             new MMPIDCommand(
//                     MMRobot.getInstance().mmSystems.elevator,
//                     73
//             )
//     );
//
//     MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//             new MMPIDCommand(
//                     MMRobot.getInstance().mmSystems.elevator,
//                     42
//             )
//     );
//
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new MMPIDCommand(
//                        MMRobot.getInstance().mmSystems.elevator,
//                        1
//                )
//        );
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                new Scoring(
//                        MMRobot.getInstance().mmSystems.elevator,
//                        MMRobot.getInstance().mmSystems.scoringArm,
//                        MMRobot.getInstance().mmSystems.claw,
//                        40
//
//                )
//        );

    }

    @Override
    public void run() {
        super.run();

//        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
//        telemetry.addData("yaw", MMRobot.getInstance().mmSystems.imu.getYawInDegrees());
//        telemetry.update();

        telemetry.addData("meow", MMRobot.getInstance().mmSystems.linearIntake.getPosition());
        telemetry.update();

/*
        telemetry.addData("meow",MMRobot.getInstance().mmSystems.elevator.getHeight());
        telemetry.addData("Ticks",MMRobot.getInstance().mmSystems.elevator.motorLeftEncoder.getCounts());
        telemetry.update();



        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();
        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();
       FtcDashboard.getInstance().getTelemetry().addData("height",MMRobot.getInstance().mmSystems.elevator.getHeight());
       FtcDashboard.getInstance().getTelemetry().update();
*/


    }

}