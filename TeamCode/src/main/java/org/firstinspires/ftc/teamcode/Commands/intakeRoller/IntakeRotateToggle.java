package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeRotateToggle extends CommandBase {
    public static boolean rollerState;
    public Intake.Roller intakeRoller;
    public IntakeRotateToggle(Intake.Roller intakeRoller) {
        this.intakeRoller = intakeRoller;
    }

    @Override
    public void initialize() {
        new IntakeRotate(intakeRoller, rollerState ? 0 : intakeRoller.COLLECT_POWER).schedule();
        rollerState = !rollerState;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
