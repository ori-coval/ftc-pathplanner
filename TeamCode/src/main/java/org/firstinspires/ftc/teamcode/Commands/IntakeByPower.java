package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.RollerIntake;

public class IntakeByPower extends InstantCommand {
    double power;

    public IntakeByPower(double power){
        this.power = power;
        this.addRequirements(MMRobot.getInstance().mmSystems.rollerIntake);
    }

    @Override
    public void initialize() {
        MMRobot.getInstance().mmSystems.rollerIntake.setPower(power);
    }
}
