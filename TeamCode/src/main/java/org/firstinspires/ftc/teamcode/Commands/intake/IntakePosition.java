package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakePosition extends ConditionalCommand {

    public IntakePosition(InTake inTake){
        super(
              new SequentialCommandGroup(
                      new WaitCommand(10),
                      new InstantCommand(()->inTake.setStackPosition(2)),
                      new WaitCommand(10),
                      new InstantCommand(()->inTake.setStackPosition(1)),
                      new WaitCommand(10),
                      new InstantCommand(()->inTake.setStackPosition(0))

              ),
              new InstantCommand(()->inTake.setStackPosition(3)),
                ()-> inTake.getStackPosition() != 3
        );
    }

}
