package org.firstinspires.ftc.teamcode.MMLib;

import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.function.Consumer;

/**
 * this command is supposed to help u avoid like maybe one line of duplicated code.
 * very useless. it was meant for the toggleWhenActive.
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
