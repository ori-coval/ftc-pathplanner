package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ShootBySupplier;
import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

import java.util.function.BooleanSupplier;


@TeleOp(name = "Experimenting")
public class ExperimentingTeleOp extends MMTeleOp {

    MMRobot mmRobot = MMRobot.getInstance();

    static BooleanSupplier isActive;

    @Override
    public void main() {
        mmRobot.mmSystems.shooter = new Shooter();

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(
                new SequentialCommandGroup(
                        new ShootBySupplier(mmRobot.mmSystems.gamepadEx1::getRightY),
                        new InstantCommand(() ->
                                isActive = () -> !isActive.getAsBoolean()
                        )
                )
        );

    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("isActive", isActive.getAsBoolean()); //I wanted to know if when I update the old telemetry gets deleted
        telemetry.update();

    }
}
