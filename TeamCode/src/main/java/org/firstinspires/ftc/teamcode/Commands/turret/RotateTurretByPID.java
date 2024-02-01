package org.firstinspires.ftc.teamcode.Commands.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class RotateTurretByPID extends CommandBase {
    private double setPoint;
    private PIDController pidController;
    private Turret turret;
    public RotateTurretByPID(Turret turret, double setPoint){
        this.setPoint = setPoint;
        this.turret = turret;
        pidController = turret.getPidController();
        pidController.setTolerance(1.5);
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        pidController.setSetPoint(setPoint);
    }

    @Override
    public void execute() {
        turret.setPower(pidController.calculate(turret.getAngle()));
        FtcDashboard.getInstance().getTelemetry().addData("Turret is finished", isFinished());
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetPoint();
    }
}
