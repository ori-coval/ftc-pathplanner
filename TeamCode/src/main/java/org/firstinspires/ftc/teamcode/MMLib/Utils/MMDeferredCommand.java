package org.firstinspires.ftc.teamcode.MMLib.Utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.Supplier;

public class MMDeferredCommand extends CommandBase {

    private final Command nullCommand = new InstantCommand();

    private final Supplier<Command> commandSupplier;
    private Command command = nullCommand;


    public MMDeferredCommand(Supplier<Command> commandSupplier, Subsystem... requireSubsystems) {
        this.commandSupplier = MMUtils.requireNonNullParam(commandSupplier, "Supplier<Command>", "MMDeferredCommand");
        addRequirements(requireSubsystems);
    }

    @Override
    public void initialize() {
        Command cmd = commandSupplier.get();
        if(cmd != null) command = cmd;
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
        command = nullCommand;
    }
}
