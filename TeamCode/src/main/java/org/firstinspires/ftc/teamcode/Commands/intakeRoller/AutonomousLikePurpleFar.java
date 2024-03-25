package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class AutonomousLikePurpleFar extends SequentialCommandGroup {
    public AutonomousLikePurpleFar(Intake.Roller intakeRoller) {
        super(new IntakeRotate(intakeRoller, intakeRoller.PURPLE_PIXEL_FAR_RED_POWER).withTimeout(300));
    }
}
