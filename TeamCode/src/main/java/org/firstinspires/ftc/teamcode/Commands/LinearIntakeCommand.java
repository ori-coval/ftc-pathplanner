package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MMRobot;

public class LinearIntakeCommand extends CommandBase{
    Trigger trigger;

    public LinearIntakeCommand(Trigger trigger){
        this.trigger = trigger;
        addRequirements(MMRobot.getInstance().mmSystems.linearIntake);
    }


    @Override
    public void execute() {
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
    }

    @Override
    public boolean isFinished() {
        return !trigger.get();
    }

    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);
    }
}
