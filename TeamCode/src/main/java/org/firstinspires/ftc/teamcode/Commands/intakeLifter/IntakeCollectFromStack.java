package org.firstinspires.ftc.teamcode.Commands.intakeLifter;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeCollectFromStack extends ParallelDeadlineGroup {
    private static final long waitTime = 500;
    public IntakeCollectFromStack(Intake.Lifter inTakeLifter, Intake.Roller inTakeRoller) {
        super(
                new IntakeUntilFull(inTakeRoller), //Deadline
                new SequentialCommandGroup(
                        new IntakeSetStackPosition(inTakeLifter, Intake.LifterPosition.FIRST_PIXEL)
/*                        new WaitCommand(waitTime),
                        new IntakeSetStackPosition(inTakeLifter, Intake.LifterPosition.SECOND_PIXEL),
                        new WaitCommand(waitTime * 3),
                        new IntakeSetStackPosition(inTakeLifter, Intake.LifterPosition.DEFAULT)*/
                )
        );
    }
}
