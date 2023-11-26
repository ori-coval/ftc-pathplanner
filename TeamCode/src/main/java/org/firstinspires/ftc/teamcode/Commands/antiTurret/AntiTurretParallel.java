package org.firstinspires.ftc.teamcode.Commands.antiTurret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;

import java.util.function.DoubleSupplier;

public class AntiTurretParallel extends CommandBase {
    AntiTurret antiTurret;
    DoubleSupplier turretAngle;
    public AntiTurretParallel(AntiTurret antiTurret, DoubleSupplier turretAngle){
        this.antiTurret = antiTurret;
        this.turretAngle = turretAngle;
        addRequirements(antiTurret);
    }

    @Override
    public void execute() {
        antiTurret.setPosition(turretAngle.getAsDouble());
    }
}
