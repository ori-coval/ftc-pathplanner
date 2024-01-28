package org.firstinspires.ftc.teamcode.Commands.intakeElbow;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeCollectFromStack extends ParallelRaceGroup {
    private static final long waitTime = 100;
    public IntakeCollectFromStack(InTake inTake){
        super(
                new IntakeUntilFull(inTake),
                new WaitCommand(waitTime),
                new IntakeSetStackPosition(inTake, 2),
                new WaitCommand(waitTime),
                new IntakeSetStackPosition(inTake, 1),
                new WaitCommand(waitTime),
                new IntakeSetStackPosition(inTake, 0)
        );
    }
}
