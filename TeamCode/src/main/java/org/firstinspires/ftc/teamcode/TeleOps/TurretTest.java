package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

@TeleOp
public class TurretTest extends CommandOpMode {
    Turret turret;

    @Override
    public void initialize() {
        turret = new Turret(hardwareMap);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new RotateTurretByPID(turret, 90));
        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new RotateTurretByPID(turret, 0));
        gamepadEx.getGamepadButton(GamepadKeys.Button.X).whenPressed(new RotateTurretByPID(turret, -90));
    }
}
