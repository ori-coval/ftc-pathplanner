package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ShootBySupplier;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp(name = "Teleop")
public class BasicTeleOp extends CommandOpMode {

    MMRobot mmRobot = MMRobot.getInstance();

    @Override
    public void initialize() {

        mmRobot.init(OpModeType.TELEOP, hardwareMap, gamepad1, gamepad2, telemetry);

        mmRobot.mmSystems.shooter.setDefaultCommand(new CommandBase() {
            {
                addRequirements(mmRobot.mmSystems.shooter);
            }
            @Override
            public void execute() {
                mmRobot.mmSystems.shooter.setPower(0.5);
            }
        });

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileActiveOnce(
                new ShootBySupplier(mmRobot.mmSystems.gamepadEx1::getLeftY)
        );

    }

    @Override
    public void run() {
        super.run();

        telemetry.addLine(String.valueOf(mmRobot.mmSystems.opModeType));
        telemetry.update();

    }
}