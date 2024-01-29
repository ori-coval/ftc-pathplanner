package org.firstinspires.ftc.teamcode.Commands.intakeLifter;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeCollectFromStack extends ParallelDeadlineGroup {
    private static final long waitTime = 250;
    public IntakeCollectFromStack(InTake.Lifter inTakeLifter, InTake.Roller inTakeRoller){
        super(
                new IntakeUntilFull(inTakeRoller), //Deadline
                new SequentialCommandGroup(
                        new IntakeSetStackPosition(inTakeLifter, 2),
                        new WaitCommand(waitTime),
                        new IntakeSetStackPosition(inTakeLifter, 1),
                        new WaitCommand(waitTime),
                        new IntakeSetStackPosition(inTakeLifter, 0),
                        new WaitCommand(waitTime * 4),
                        new IntakeSetStackPosition(inTakeLifter, 4)
                )
        );
    }
}
