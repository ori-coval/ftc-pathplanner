package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeRotate extends CommandBase {
    private Intake.Roller inTakeRoller;
    private final double power;

    public IntakeRotate(Intake.Roller inTakeRoller, double power) {
        this.inTakeRoller = inTakeRoller;
        this.power = power;
        this.addRequirements(inTakeRoller);
    }


    @Override
    public void initialize() {
        inTakeRoller.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        inTakeRoller.stop();
    }

}



