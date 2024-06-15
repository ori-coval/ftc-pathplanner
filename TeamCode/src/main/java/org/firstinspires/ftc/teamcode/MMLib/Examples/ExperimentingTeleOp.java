package org.firstinspires.ftc.teamcode.MMLib.Examples;

import android.hardware.TriggerEvent;
import android.hardware.TriggerEventListener;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Commands.ShootBySupplier;
import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

import java.util.function.BooleanSupplier;


@TeleOp(name = "Experimenting")
public class ExperimentingTeleOp extends MMTeleOp {

    MMRobot mmRobot = MMRobot.getInstance();

    boolean isActive = false;

    BooleanSupplier isActiveSupplier = this::getIsActive;

    public ExperimentingTeleOp() {
        super(false);
    }

    public boolean getIsActive() {
        return isActive;
    }

    @Override
    public void main() {
        mmRobot.mmSystems.shooter = new Shooter();

        Runnable changeStage = () -> isActive = !isActiveSupplier.getAsBoolean();

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(
                new ShootBySupplier(mmRobot.mmSystems.gamepadEx1::getRightY).alongWith(
                        new StartEndCommand(
                                changeStage,
                                changeStage
                        )
                )
        );

    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("isActive", isActiveSupplier.getAsBoolean());
        telemetry.update();

    }
}
