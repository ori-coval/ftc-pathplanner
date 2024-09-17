package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MMRobot;

public class LinearIntakeCommand extends CommandBase{

    MMRobot robot = MMRobot.getInstance();

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        robot.mmSystems.linearIntake.setPosition(robot.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)/4);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        robot.mmSystems.linearIntake.setPosition(0);
    }
}
