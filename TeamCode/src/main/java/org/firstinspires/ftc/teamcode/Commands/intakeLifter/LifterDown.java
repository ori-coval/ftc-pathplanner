package org.firstinspires.ftc.teamcode.Commands.intakeLifter;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class LifterDown extends InstantCommand {
    public LifterDown(Intake.Lifter intakeLifter) {
        super(() -> {
            if(LifterUp.currentValue != LifterUp.lifterPositions.length - 1) {
                intakeLifter.setPosition(LifterUp.lifterPositions[++LifterUp.currentValue]);
            }
        });
    }

}
