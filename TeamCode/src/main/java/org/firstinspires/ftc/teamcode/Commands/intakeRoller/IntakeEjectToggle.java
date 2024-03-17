package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetLifterPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeEjectToggle extends CommandBase {
    private RobotControl robot;
    public IntakeEjectToggle(RobotControl robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        new IntakeRotate(robot.intake.roller, IntakeRotateToggle.isRollerActive ? 0 : robot.intake.roller.EJECT_POWER).schedule();
        if(IntakeRotateToggle.isRollerActive) {
            new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT).schedule();
        }
        IntakeRotateToggle.isRollerActive = !IntakeRotateToggle.isRollerActive;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
