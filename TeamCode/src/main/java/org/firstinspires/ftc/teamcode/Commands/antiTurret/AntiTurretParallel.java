package org.firstinspires.ftc.teamcode.Commands.antiTurret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;

import java.util.function.DoubleSupplier;

public class AntiTurretParallel extends CommandBase {
    private AntiTurret antiTurret;
    private DoubleSupplier turretAngle;
    public AntiTurretParallel(AntiTurret antiTurret, DoubleSupplier turretAngle){
        this.antiTurret = antiTurret;
        this.turretAngle = turretAngle;
        addRequirements(antiTurret);
    }
    @Override
    public void execute() {
        antiTurret.setPos(turretAngle.getAsDouble());
    }
}
