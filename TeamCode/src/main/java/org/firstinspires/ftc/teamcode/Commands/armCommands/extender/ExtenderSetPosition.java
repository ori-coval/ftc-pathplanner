package org.firstinspires.ftc.teamcode.Commands.armCommands.extender;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Extender;

public class ExtenderSetPosition extends InstantCommand {
    public ExtenderSetPosition(Extender extender, Extender.Position position) {
        super(() -> extender.setPosition(position), extender);
    }
}
