package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.DoubleSupplier;

public class LinearIntakeCommand extends CommandBase{
    Trigger trigger;
    DoubleSupplier position;

    public LinearIntakeCommand(Trigger trigger, DoubleSupplier position){
        this.position = position;
        this.trigger = trigger;
        addRequirements(MMRobot.getInstance().mmSystems.linearIntake);
    }


    @Override
    public void execute() {
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(position.getAsDouble());
    }

    @Override
    public boolean isFinished() {
//        return false;
        return !trigger.get();
    }

    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);
    }
}
