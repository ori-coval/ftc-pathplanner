package org.firstinspires.ftc.teamcode.MMLib;

import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.function.Consumer;

/**
 * this command is supposed to help u avoid like maybe one line of duplicated code.
 * very useless. it was meant for the toggleWhenActive.
 * @param <T> the type ur setPower gets
 */
public class MMToggleCommand2<T> extends StartEndCommand {
    public MMToggleCommand2(MMPowerSubsystem<T> subsystem, T on, T off, Subsystem... requirements) {
        super(
                () -> subsystem.setPower(on),
                () -> subsystem.setPower(off),
                requirements
        );
    }
}
