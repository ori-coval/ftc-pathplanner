package org.firstinspires.ftc.teamcode.MMLib.Examples.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MMLib.Examples.Commands.ShootByPID;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMLib.Examples.Subsystems.ShooterPID;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp(name = "Teleop")
public class BasicTeleOp extends CommandOpMode {

    MMRobot mmRobot = MMRobot.getInstance();

    @Override
    public void initialize() {
        mmRobot.init(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION, hardwareMap, gamepad1, gamepad2, telemetry);

        mmRobot.mmSystems.shooterPID = new ShooterPID();

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ShootByPID(-5)
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ShootByPID(0)
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ShootByPID(5)
        );

    }

    @Override
    public void run() {
        super.run();
        FtcDashboard.getInstance().getTelemetry().update();
        telemetry.update();
    }
}