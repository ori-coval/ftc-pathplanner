package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakePosition extends ConditionalCommand {
    public IntakePosition(InTake inTake) {
        super(
                new CollectingFromStack(inTake),
                new IntakeSetStackPosition(inTake, 3),
                () -> inTake.getStackPosition() == 3

        );
    }

//    private static CommandBase intakeCommand(InTake inTake) {
//        SequentialCommandGroup group = new SequentialCommandGroup();
//        for (int i = 2; i >= 0; i--) {
//            int finalI = i;
//            group.addCommands(
//                    new WaitCommand(waitTime),
//                    new InstantCommand(() -> inTake.setStackPosition(finalI))
//            );
//        }
//        return group;
//    }

}
