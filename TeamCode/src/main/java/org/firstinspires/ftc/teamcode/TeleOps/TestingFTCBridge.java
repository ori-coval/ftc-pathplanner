package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;

@TeleOp
public class TestingFTCBridge extends MMTeleOp {

    MMRobot mmRobot = MMRobot.getInstance();

    public TestingFTCBridge() {
        super(false);
    }

    @Override
    public void main() {

        CuttleMotor motor = mmRobot.mmSystems.controlHub.getMotor(3);

        addRunnableOnInit(
                () -> motor.setPower(-1)
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> motor.setPower(0.5))
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> motor.setPower(0))
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> motor.setPower(-0.5))
        );

    }

}
