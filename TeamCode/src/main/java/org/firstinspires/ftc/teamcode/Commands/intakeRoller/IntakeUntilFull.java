package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
public class IntakeUntilFull extends ParallelDeadlineGroup {
    private InTake intake;

    public IntakeUntilFull(InTake intake) {
        super(new WaitUntilCommand(()->intake.isRobotFull()), new IntakeRotate(intake, intake.COLLECT_POWER));
    }
}
