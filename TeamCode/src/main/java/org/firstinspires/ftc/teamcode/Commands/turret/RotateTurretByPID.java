package org.firstinspires.ftc.teamcode.Commands.turret;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class RotateTurretByPID extends CommandBase {
    private double setPoint;
    private PIDController pidController;
    private Turret turret;
    public RotateTurretByPID(Turret turret, double setPoint){
        this.setPoint= setPoint;
        this.turret = turret;
        addRequirements(turret);

    }

    @Override
    public void initialize() {
        pidController.setSetPoint(setPoint);
        pidController.setTolerance(0.08);
    }

    @Override
    public void execute() {
        turret.setPower(pidController.calculate(turret.getAngle()));
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

}
