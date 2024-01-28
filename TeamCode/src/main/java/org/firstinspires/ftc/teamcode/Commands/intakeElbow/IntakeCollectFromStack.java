package org.firstinspires.ftc.teamcode.Commands.intakeElbow;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeCollectFromStack extends ParallelRaceGroup {
    private static final long waitTime = 100;
    public IntakeCollectFromStack(InTake.Lifter inTakeLifter, InTake.Roller inTakeRoller){
        super(
                new IntakeUntilFull(inTakeRoller),
                new SequentialCommandGroup(new IntakeSetStackPosition(inTakeLifter, 2),
                new WaitCommand(waitTime),
                new IntakeSetStackPosition(inTakeLifter, 1),
                new WaitCommand(waitTime),
                new IntakeSetStackPosition(inTakeLifter, 0)
                )
        );
    }
}
