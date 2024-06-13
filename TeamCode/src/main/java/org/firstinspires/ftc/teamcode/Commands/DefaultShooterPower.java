package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

public class DefaultShooterPower extends CommandBase {

    Shooter shooter;

    public DefaultShooterPower(Shooter shooter) {
        this.shooter = shooter;
        this.addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setPower(0.5);
    }
}
