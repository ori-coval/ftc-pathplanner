package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShootBySupplier extends CommandBase {

    Shooter shooter;
    DoubleSupplier power;
    public ShootBySupplier(Shooter shooter, DoubleSupplier power){
        this.shooter = shooter;
        this.power = power;
        this.addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setPower(power.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPower(0);
    }
}
