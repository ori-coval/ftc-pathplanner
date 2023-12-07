package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Commands.Conveyer.ConveyorConvey;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeRotate;
import org.firstinspires.ftc.teamcode.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeUntilFull extends ParallelDeadlineGroup {
    private InTake inTake;
    private Conveyor conveyor;

    public IntakeUntilFull(InTake inTake, Conveyor conveyor) {
        super(
                new WaitUntilCommand(()->(conveyor.getPixelCount() >= 2)),
                new IntakeRotate(inTake),
                new ConveyorConvey(conveyor)
        );
    }


}




