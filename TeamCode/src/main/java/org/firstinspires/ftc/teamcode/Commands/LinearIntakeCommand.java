package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;

public class LinearIntakeCommand extends CommandBase{

    MMRobot robot = MMRobot.getInstance();
    LinearIntake linearIntake;

    @Override
    public void execute() {
        robot.mmSystems.linearIntake.setPosition(robot.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
   //TODO: tuning
    }

    @Override
    public void end(boolean interrupted) {
        linearIntake.setPosition(0);
    }
}
