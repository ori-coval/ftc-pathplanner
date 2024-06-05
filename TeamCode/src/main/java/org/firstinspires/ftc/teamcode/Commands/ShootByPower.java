package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

public class ShootByPower extends CommandBase {

    Shooter shooter;
    double power;
    public ShootByPower(Shooter shooter, double power){
        this.shooter = shooter;
        this.power = power;
        this.addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPower(0);
    }
}
