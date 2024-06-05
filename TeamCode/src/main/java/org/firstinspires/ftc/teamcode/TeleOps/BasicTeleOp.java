package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop")
public class BasicTeleOp extends CommandOpMode {

    // subsystems here

    @Override
    public void initialize() {
        //init gamepad
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        //init subsystem
        //Ex: subsystem  = new Subsystem(hardwareMap)

        //setup buttons
        //Ex: gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenActive(new SomethingCommand());

        //create new triggers
        //Ex: Trigger newTrigger = new Trigger(() -> gamepadEx1.getLeftY() > 0.5);
    }

    @Override
    public void run() {
        super.run();

        //setting up telemetry
        //telemetry.addData("name", value)
        telemetry.update();
    }
}