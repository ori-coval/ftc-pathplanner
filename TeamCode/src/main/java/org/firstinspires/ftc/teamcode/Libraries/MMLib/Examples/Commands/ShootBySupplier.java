package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.DoubleSupplier;

public class ShootBySupplier extends CommandBase {

    Shooter shooter = MMRobot.getInstance().mmSystems.shooter;
    DoubleSupplier power;


    public ShootBySupplier(DoubleSupplier power){
        this.power = power;
        this.addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setPower(power.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPower(0.);
    }
}
