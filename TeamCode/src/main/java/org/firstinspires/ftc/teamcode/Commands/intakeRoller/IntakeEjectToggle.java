package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeEjectToggle extends CommandBase {
    public static boolean rollerState;
    public Intake.Roller intakeRoller;
    public IntakeEjectToggle(Intake.Roller intakeRoller) {
        this.intakeRoller = intakeRoller;
    }

    @Override
    public void initialize() {
        new IntakeRotate(intakeRoller, rollerState ? 0 : intakeRoller.EJECT_POWER).schedule();
        rollerState = !rollerState;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
