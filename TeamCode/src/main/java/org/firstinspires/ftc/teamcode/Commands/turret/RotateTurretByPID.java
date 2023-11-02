package org.firstinspires.ftc.teamcode.Commands.turret;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class RotateTurretByPID extends CommandBase {
    double setPoint;
    PIDController pidController;
    Turret turret;
    public RotateTurretByPID(double setPoint, Turret turret){
        this.setPoint= setPoint;
        pidController = new PIDController(1,0,0);
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
        turret.setPower(pidController.calculate(turret.getEncoderVoltage()));
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
