package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.conveyer.ConveyorConvey;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeRotate;
import org.firstinspires.ftc.teamcode.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeUntilFull extends SequentialCommandGroup {
    private InTake inTake;
    private Conveyor conveyor;

    public IntakeUntilFull(InTake inTake, Conveyor conveyor) {
        super(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(conveyor::isRobotFull),
                        new IntakeRotate(inTake, inTake.COLLECT_POWER),
                        new ConveyorConvey(conveyor, conveyor.IN_POWER)
                ),
                new ParallelCommandGroup(
                        new IntakeRotate(inTake, inTake.EJECT_POWER),
                        new ConveyorConvey(conveyor, conveyor.OUT_POWER)
                ).withTimeout(1500)
        );
    }


}




