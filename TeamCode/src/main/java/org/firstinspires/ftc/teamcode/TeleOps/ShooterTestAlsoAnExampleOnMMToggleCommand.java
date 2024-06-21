package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.RotateTurretByPid;
import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMMotor;
import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMLib.MMToggleCommand;
import org.firstinspires.ftc.teamcode.MMLib.MMToggleCommand2;
import org.firstinspires.ftc.teamcode.MMLib.Subsystems.IMMPositionSubsystem;
import org.firstinspires.ftc.teamcode.MMLib.Subsystems.IMMPowerSubsystem;
import org.firstinspires.ftc.teamcode.MMLib.Subsystems.MMPowerSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.ShooterIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ShooterTurret;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

@TeleOp(name = "Nimrod's Shooter")
public class ShooterTestAlsoAnExampleOnMMToggleCommand extends MMTeleOp {

    public ShooterTestAlsoAnExampleOnMMToggleCommand() {
        super(false);
    }

    MMRobot mmRobot = MMRobot.getInstance();

    @Override
    public void main() {

        mmRobot.mmSystems.shooter = new Shooter();
        mmRobot.mmSystems.shooterIntake = new ShooterIntake();
        mmRobot.mmSystems.shooterTurret = new ShooterTurret();


        //Turret

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileActiveOnce(
                new MMToggleCommand2<>(
                        mmRobot.mmSystems.shooterTurret,
                        0.5, 0.
                )
        );

/*        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new RotateTurretByPid(30)
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new RotateTurretByPid(0)
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new RotateTurretByPid(-30)
        );*/


        //Shooter and Intake

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(
                new MMToggleCommand2<>(
                        (IMMPowerSubsystem<Double>) mmRobot.mmSystems.shooter,
                        1., 0.,
                        mmRobot.mmSystems.shooter
                )
        );


        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).toggleWhenActive(
                new MMToggleCommand2<>(
                        (IMMPositionSubsystem<Double>) mmRobot.mmSystems.shooter,
                        0.2, 0.
                )
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(
                new MMToggleCommand<>(
                        (x) -> {
                            mmRobot.mmSystems.shooterIntake.setMotorPower(x);
                            mmRobot.mmSystems.shooterIntake.setServoPower(x);
                        },
                        1., 0.,
                        mmRobot.mmSystems.shooterIntake
                )

        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).toggleWhenActive(
                new MMToggleCommand<>(
                        mmRobot.mmSystems.shooterIntake::setMotorPower,
                        1., 0.,
                        mmRobot.mmSystems.shooterIntake
                )
        );

    }
}
