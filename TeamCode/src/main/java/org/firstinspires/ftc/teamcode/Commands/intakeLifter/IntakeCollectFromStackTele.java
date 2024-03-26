package org.firstinspires.ftc.teamcode.Commands.intakeLifter;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.intakeRoller.InstantIntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotateToggle;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeCollectFromStackTele extends SequentialCommandGroup {
    public IntakeCollectFromStackTele(RobotControl robot) {
        super(
                IntakeRotateToggle.intakeArmCommand(robot),
                new InstantIntakeRotate(robot, robot.intake.roller.COLLECT_POWER),
                new InstantCommand(() -> IntakeRotateToggle.isRollerActive = true),
                new SequentialCommandGroup(
                        new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL),
                        new WaitCommand(1000),
                        new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.SECOND_PIXEL)
                )
        );
    }
}
