package org.firstinspires.ftc.teamcode.MMLib.Examples.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMLib.Utils.MMToggleCommand;
import org.firstinspires.ftc.teamcode.MMLib.Utils.MMToggleCommand2;
import org.firstinspires.ftc.teamcode.MMLib.SubsystemStructure.IMMPositionSubsystem;
import org.firstinspires.ftc.teamcode.MMLib.SubsystemStructure.IMMPowerSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMLib.Examples.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.MMLib.Examples.Subsystems.ShooterIntake;
import org.firstinspires.ftc.teamcode.MMLib.Examples.Subsystems.ShooterTurret;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp(name = "Nimrod's Shooter")
public class ShooterTestAlsoAnExampleOnMMToggleCommand extends MMTeleOp {

    public ShooterTestAlsoAnExampleOnMMToggleCommand() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    MMRobot mmRobot = MMRobot.getInstance();

    @Override
    public void onInit() {

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
