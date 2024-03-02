package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeRotateToggle extends CommandBase {
    public static boolean isRollerActive;
    RobotControl robot;
    public IntakeRotateToggle(RobotControl robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        new IntakeRotate(robot.intake.roller, isRollerActive ? 0 : robot.intake.roller.COLLECT_POWER).schedule();
        if(isRollerActive) {
            new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT).schedule();
        }
        isRollerActive = !isRollerActive;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
