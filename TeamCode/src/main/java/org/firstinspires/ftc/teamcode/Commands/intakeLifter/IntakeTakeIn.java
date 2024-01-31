package org.firstinspires.ftc.teamcode.Commands.intakeLifter;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeTakeIn extends ConditionalCommand {
    public IntakeTakeIn(InTake.Lifter inTakeLifter, InTake.Roller inTakeRoller) {
        super(
                new IntakeCollectFromStack(inTakeLifter, inTakeRoller),
                new IntakeSetStackPosition(inTakeLifter, 3),
                () -> inTakeLifter.getStackPosition() == 3

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
