package org.firstinspires.ftc.teamcode.Commands.intakeLifter;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeTakeIn extends ConditionalCommand {
    public IntakeTakeIn(Intake.Lifter inTakeLifter, Intake.Roller inTakeRoller) {
        super(
                new IntakeCollectFromStack(inTakeLifter, inTakeRoller),
                new IntakeSetStackPosition(inTakeLifter, 3),
                () -> inTakeLifter.getStackPosition() == 3

        );
    }

}
