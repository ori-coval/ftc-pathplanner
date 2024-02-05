package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeUntilFull extends SequentialCommandGroup {
    private Intake intake;

    public IntakeUntilFull(Intake intake) {
        super(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(intake.roller::isRobotFull),
                        new IntakeRotate(intake.roller, intake.roller.COLLECT_POWER)
                ),
                new ParallelCommandGroup(
                        new IntakeRotate(intake.roller, intake.roller.EJECT_POWER)
                ).withTimeout(1500)
                );
    }


}




