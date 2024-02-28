package org.firstinspires.ftc.teamcode.Commands.armCommands.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

import java.net.HttpURLConnection;
import java.util.Calendar;

public class RotateTurretByPID extends CommandBase {
    private final double setPoint;
    private final PIDController pidController;
    private RobotControl robot;
    Turret turret;
    private long startTime;
    private final long TIME_WAITING_FOR_ELBOW = 4000;
    public RotateTurretByPID(Turret turret, double setPoint){
        this.setPoint = setPoint;
        this.turret = turret;
        pidController = turret.getPidController();
        pidController.setTolerance(0.5);
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        pidController.setSetPoint(setPoint);
        startTime = Calendar.getInstance().getTimeInMillis();
    }

    @Override
    public void execute() {
        if(robot.elbow.isInSafePlace() || !robot.turret.isListeningToElbowSensor) {
            robot.turret.setPower(pidController.calculate(robot.turret.getAngle()));
        } else if(Calendar.getInstance().getTimeInMillis() - startTime > TIME_WAITING_FOR_ELBOW) {
            pidController.setSetPoint(robot.turret.getAngle());
        }
        FtcDashboard.getInstance().getTelemetry().addData("Turret is finished", isFinished());
    }

    @Override
    public boolean isFinished() {
        if(pidController.atSetPoint()) {
            return Calendar.getInstance().getTimeInMillis() - startTime > 2000;
        } else {
            startTime = Calendar.getInstance().getTimeInMillis();
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

}
