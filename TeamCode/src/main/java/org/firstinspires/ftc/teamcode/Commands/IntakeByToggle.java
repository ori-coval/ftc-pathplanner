package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeByToggle extends CommandBase {
    Intake intake = MMRobot.getInstance().mmSystems.intake;

    public IntakeByToggle(){
        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPower(1);
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(intake.setPower(0));
    }



}
