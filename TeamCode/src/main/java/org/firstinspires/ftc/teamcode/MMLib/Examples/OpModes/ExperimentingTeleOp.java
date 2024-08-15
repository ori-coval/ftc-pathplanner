package org.firstinspires.ftc.teamcode.MMLib.Examples.OpModes;

import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MMLib.Examples.Commands.ShootBySupplier;
import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMLib.Examples.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

import java.util.function.BooleanSupplier;


@TeleOp(name = "Experimenting")
public class ExperimentingTeleOp extends MMTeleOp {

    MMRobot mmRobot = MMRobot.getInstance();

    boolean isActive = false;

    BooleanSupplier isActiveSupplier = this::getIsActive;

    public ExperimentingTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    public boolean getIsActive() {
        return isActive;
    }

    @Override
    public void onInit() {
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
        telemetry.addData("isActive", isActiveSupplier.getAsBoolean());
        telemetry.update();
    }
}
