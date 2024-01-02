package org.firstinspires.ftc.teamcode.Commands.positionSelector;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.ArmPositionSelector;

public class PositionSelectorXRight extends InstantCommand {
    public PositionSelectorXRight() {
        super(() -> ArmPositionSelector.moveXRight());
    }
}
