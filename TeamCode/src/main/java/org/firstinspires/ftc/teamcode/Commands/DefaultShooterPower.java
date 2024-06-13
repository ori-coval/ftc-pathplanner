package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

public class DefaultShooterPower extends CommandBase {

    Shooter shooter = MMRobot.getInstance().mmSystems.shooter;

    public DefaultShooterPower() {
        this.addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setPower(0.5);
    }
}
