package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeByToggle extends CommandBase {
    Intake intake = MMRobot.getInstance().mmSystems.intake;

    public IntakeByToggle() {
        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (intake.isRoll) {
            intake.setPower(0);
            intake.isRoll = false;
        } else {
            intake.setPower(1);
            intake.isRoll = true;
        }
    }


}
