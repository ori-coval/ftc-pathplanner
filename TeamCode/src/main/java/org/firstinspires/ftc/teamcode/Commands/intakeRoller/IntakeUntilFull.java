package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
public class IntakeUntilFull extends ParallelDeadlineGroup {
    private static final long waitTimeUntilStop = 1000;
    public IntakeUntilFull(InTake.Roller intakeRoller) {
        super(
                new SequentialCommandGroup(
                        new WaitUntilCommand(intakeRoller::isRobotFull),
                        new WaitCommand(waitTimeUntilStop)
                ).asProxy(),
                new IntakeRotate(intakeRoller, intakeRoller.COLLECT_POWER)
        );
    }
}

/*
package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
public class IntakeUntilFull extends CommandBase {
    private static final long waitTimeUntilStop = 2000;
    ParallelDeadlineGroup parallelDeadlineGroup;
    public IntakeUntilFull(InTake.Roller intakeRoller) {
        SequentialCommandGroup deadlineWaitCommand = new SequentialCommandGroup(
                new WaitUntilCommand(intakeRoller::isRobotFull),
                new WaitCommand(waitTimeUntilStop)
        );
        parallelDeadlineGroup = new ParallelDeadlineGroup(
                deadlineWaitCommand,
                new IntakeRotate(intakeRoller, intakeRoller.COLLECT_POWER)
        );
    }

    @Override
    public void initialize() {
        parallelDeadlineGroup.schedule();
    }
}

 */