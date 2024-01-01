package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeSetPosition extends InstantCommand {
    public IntakeSetPosition(InTake inTake, int pos) {
        super(() -> inTake.setStackPosition(pos), inTake);
    }
}
