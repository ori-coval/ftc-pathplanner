package org.firstinspires.ftc.teamcode.Commands.armCommands.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotControl;

import java.net.HttpURLConnection;
import java.util.Calendar;

public class RotateTurretByPID extends CommandBase {
    private double setPoint;
    private PIDController pidController;
    private RobotControl robot;
    private long startTime;
    private final long TIME_WAITING_FOR_ELBOW = 4000;

    public RotateTurretByPID(RobotControl robot, double setPoint){
        this.setPoint = setPoint;
        this.robot = robot;
        pidController = robot.turret.getPidController();
        pidController.setTolerance(0.5);
        addRequirements(robot.turret);
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
            setPoint = robot.turret.getAngle();
        }
        FtcDashboard.getInstance().getTelemetry().addData("Turret is finished", isFinished());
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        robot.turret.stop();
    }

}
