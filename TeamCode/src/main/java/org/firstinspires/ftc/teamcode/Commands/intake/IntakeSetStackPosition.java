package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeSetStackPosition extends InstantCommand {
    public IntakeSetStackPosition(InTake intake, int stackPosition) {
        super(() -> intake.setStackPosition(stackPosition));
        addRequirements(intake);
    }
}
