package org.firstinspires.ftc.teamcode.MMLib.Utils;

import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.function.Consumer;

/**
 * this command gets a consumer and an on and off state to feed to the consumer.
 * it builds a {@link StartEndCommand} where the {@link StartEndCommand#initialize() start()} method runs the on and the {@link StartEndCommand#end(boolean) end()} runs the off state.
 * @param <T> the type ur consumer gets
 */
public class MMToggleCommand<T> extends StartEndCommand {
    public MMToggleCommand(Consumer<T> set, T on, T off, Subsystem... requirements) {
        super(
                () -> set.accept(on),
                () -> set.accept(off),
                requirements
        );
    }
}
