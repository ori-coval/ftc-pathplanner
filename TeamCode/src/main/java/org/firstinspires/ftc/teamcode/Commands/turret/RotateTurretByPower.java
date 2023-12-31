package org.firstinspires.ftc.teamcode.Commands.turret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class RotateTurretByPower extends CommandBase {
    private Turret turret;
    private double power;
    public RotateTurretByPower(double power, Turret turret){
        this.power = power;
        this.turret = turret;
        addRequirements(turret);
    }
    @Override
    public void initialize() {
        turret.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
