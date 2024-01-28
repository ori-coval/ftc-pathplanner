package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class CollectingFromStack extends SequentialCommandGroup {
    private static final long waitTime = 100;
    public CollectingFromStack(InTake inTake){
        super(
                new IntakeRotate(inTake, inTake.COLLECT_POWER),
                new WaitCommand(waitTime),
                new IntakeSetStackPosition(inTake, 2),
                new WaitCommand(waitTime),
                new IntakeSetStackPosition(inTake, 1),
                new WaitCommand(waitTime),
                new IntakeSetStackPosition(inTake, 0)
        );
    }
}
