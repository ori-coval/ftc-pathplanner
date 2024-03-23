package org.firstinspires.ftc.teamcode.Commands.intakeLifter;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class LifterUp extends InstantCommand {

    public static Intake.LifterPosition[] lifterPositions = Intake.LifterPosition.values();

    public static int currentValue;

    public LifterUp(Intake.Lifter intakeLifter) {
        super(() -> {
            if (currentValue != 0) {
                intakeLifter.setPosition(lifterPositions[--currentValue]);
            }
        });
    }
}
