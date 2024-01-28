package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeTakeIn extends ConditionalCommand {
    public IntakeTakeIn(InTake inTake) {
        super(
                new IntakeCollectFromStack(inTake),
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
