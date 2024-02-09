package org.firstinspires.ftc.teamcode.Commands.intakeLifter;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeSetStackPosition extends InstantCommand {
    public IntakeSetStackPosition(Intake.Lifter intakeLifter, Intake.LifterPosition stackPosition) {
        super(() -> intakeLifter.setPosition(stackPosition), intakeLifter);
    }
}
