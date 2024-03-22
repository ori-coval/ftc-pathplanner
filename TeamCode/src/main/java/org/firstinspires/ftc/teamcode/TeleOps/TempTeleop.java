package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

@TeleOp(name = "Elevator Test")
public class TempTeleop extends CommandOpMode {

    Elevator elevator;
    Intake intake;

    @Override
    public void initialize() {

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        elevator = new Elevator(hardwareMap);
        intake = new Intake(hardwareMap);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileActiveOnce(new StartEndCommand(
                () -> intake.lifter.setPosition(Intake.LifterPosition.FIRST_PIXEL),
                () -> intake.lifter.setPosition(Intake.LifterPosition.INIT),
                intake.lifter
        ));

        Trigger elevatorTriggerUp = new Trigger(() -> gamepadEx1.getLeftY() > 0.5);
        Trigger elevatorTriggerDown = new Trigger(() -> gamepadEx1.getLeftY() < -0.5);

        elevatorTriggerUp.whileActiveOnce(new StartEndCommand(
                () -> elevator.setPower(0.9),
                elevator::stop,
                elevator
        ));

        elevatorTriggerDown.whileActiveOnce(new StartEndCommand(
                () -> elevator.setPower(-1),
                elevator::stop,
                elevator
        ));

    }
}


//startPoint_to_endPoint