package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Commands.IntakeArmSetState;
import org.firstinspires.ftc.teamcode.Commands.IntakeByPower;
import org.firstinspires.ftc.teamcode.Commands.LinearIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.SetLinearPosition;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;

import java.util.function.DoubleSupplier;

public class Intake extends ParallelDeadlineGroup {

    Trigger trigger;

    public Intake(Trigger trigger, DoubleSupplier LinearIntakePose) {
        super(
                new LinearIntakeCommand(trigger, LinearIntakePose),
                new IntakeArmSetState(IntakeArm.Position.OUT),
                new IntakeByPower(1)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        CommandScheduler.getInstance().schedule(
                new IntakeArmSetState(IntakeArm.Position.IN),
                new IntakeByPower(0),
                new SetLinearPosition(0)
        );


    }

}
