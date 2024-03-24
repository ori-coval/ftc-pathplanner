package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.StartEndCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class AutonomousLikePurpleClose extends StartEndCommand {
    public AutonomousLikePurpleClose(Intake.Roller intakeRoller) {
        super(
                () -> intakeRoller.setPower(-0.5),
                intakeRoller::stop,
                intakeRoller
        );
    }
}
