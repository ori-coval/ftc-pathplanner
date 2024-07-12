package org.firstinspires.ftc.teamcode.MMLib.Utils;

import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.MMLib.Subsystems.IMMPositionSubsystem;
import org.firstinspires.ftc.teamcode.MMLib.Subsystems.IMMPowerSubsystem;

/**
 * this command is supposed to help u avoid like maybe one line of duplicated code.
 * very useless. it was meant for the toggleWhenActive.
 * @param <T> the type of ur subsystem
 */
public class MMToggleCommand2<T> extends StartEndCommand {
    public MMToggleCommand2(IMMPowerSubsystem<T> subsystem, T on, T off, Subsystem... requirements) {
        super(
                () -> subsystem.setPower(on),
                () -> subsystem.setPower(off),
                requirements
        );
    }

    public MMToggleCommand2(IMMPositionSubsystem<T> subsystem, T on, T off, Subsystem... requirements) {
        super(
                () -> subsystem.setPosition(on),
                () -> subsystem.setPosition(off),
                requirements
        );
    }

}
