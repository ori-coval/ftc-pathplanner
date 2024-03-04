package org.firstinspires.ftc.teamcode.Commands.intakeLifter;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeTakeIn extends ConditionalCommand {
    public IntakeTakeIn(RobotControl robot) {
        super(
                new IntakeCollectFromStack(robot.intake.lifter, robot.intake.roller).andThen(
                        new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT)
                ),
                new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.STANDBY),
                () -> robot.intake.lifter.getPosition() == Intake.LifterPosition.STANDBY

        );
    }

}
