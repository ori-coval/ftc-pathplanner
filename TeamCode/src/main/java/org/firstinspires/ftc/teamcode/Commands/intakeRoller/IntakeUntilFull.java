package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;
public class IntakeUntilFull extends ParallelDeadlineGroup {
    private static final long waitTimeUntilStop = 2000;
    public IntakeUntilFull(Intake.Roller intakeRoller) {
        super(
                new SequentialCommandGroup(
                        new WaitUntilCommand(intakeRoller::isRobotFull),
                        new WaitCommand(waitTimeUntilStop)
                ).asProxy(),
                new IntakeRotate(intakeRoller, intakeRoller.COLLECT_POWER)
        );
    }
}