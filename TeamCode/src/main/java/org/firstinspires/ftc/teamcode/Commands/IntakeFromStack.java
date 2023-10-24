package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeFromStack extends SequentialCommandGroup {

    private final double startedPosition = 0;
    private final double finelAngel = 0;
    private InTake inTake;

    public IntakeFromStack(InTake inTake){
        addCommands(
          new RunCommand(()->inTake.setPosition(startedPosition)).interruptOn(()->inTake.atTarget(startedPosition)),
          new InstantCommand(()->inTake.setPower(0.85)),
          new RunCommand(()->inTake.setPosition(finelAngel)).interruptOn(()->inTake.atTarget(finelAngel)),
          new InstantCommand(()->inTake.setPower(0)),
          new RunCommand(()->inTake.setPosition(startedPosition)).interruptOn(()->inTake.atTarget(startedPosition))
        );
        this.inTake = inTake;

    }




}
