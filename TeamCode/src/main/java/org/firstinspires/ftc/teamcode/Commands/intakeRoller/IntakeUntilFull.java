package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
public class IntakeUntilFull extends ParallelDeadlineGroup {
    private InTake.Roller intakeRoller;

    public IntakeUntilFull(InTake.Roller intakeRoller) {
        super(new WaitUntilCommand(intakeRoller::isRobotFull), new IntakeRotate(intakeRoller, intakeRoller.COLLECT_POWER));
    }
}
