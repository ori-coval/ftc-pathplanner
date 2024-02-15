package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;
public class IntakeUntilFullAndEject extends ParallelDeadlineGroup {
    private static final long waitTimeUntilStop = 1000;
    public IntakeUntilFullAndEject(Intake.Roller intakeRoller) {
        super(
                new SequentialCommandGroup(
                        new WaitUntilCommand(intakeRoller::isRobotFull),
                        new WaitCommand(waitTimeUntilStop),
                        new IntakeRotate(intakeRoller, intakeRoller.EJECT_POWER).withTimeout(2 * waitTimeUntilStop)
                ).asProxy(),
                new IntakeRotate(intakeRoller, intakeRoller.COLLECT_POWER)
        );
    }
}