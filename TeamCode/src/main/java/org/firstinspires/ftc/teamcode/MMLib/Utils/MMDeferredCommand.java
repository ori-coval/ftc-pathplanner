package org.firstinspires.ftc.teamcode.MMLib.Utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.function.Supplier;

public class MMDeferredCommand extends CommandBase {

    private final Command nullCommand = new InstantCommand();

    private final Supplier<Command> supplier;
    private Command command = nullCommand;


    public MMDeferredCommand(Supplier<Command> command, Subsystem... requireSubsystems) {
        this.supplier = MMUtils.requireNonNullParam(command, "Supplier<Command>", "MMDeferredCommand");
        addRequirements(requireSubsystems);
    }

    @Override
    public void initialize() {
        Command cmd = supplier.get();
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
