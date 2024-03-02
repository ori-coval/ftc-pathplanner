package org.firstinspires.ftc.teamcode.Commands.armCommands.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

import java.util.Calendar;

public class RotateTurretByPID extends CommandBase {
    private final double setPoint;
    private final PIDController pidController;
    private final RobotControl robot;
    private long startTime;
    private final long TIME_WAITING_FOR_TURRET_PID = 100; // todo need to tune this
    private final long TIME_WAITING_FOR_ELBOW = 2000;
    public RotateTurretByPID(RobotControl robot, double setPoint){
        this.setPoint = setPoint;
        this.robot = robot;
        pidController = robot.turret.getPidController();
        pidController.setTolerance(Turret.tolerance);
        addRequirements(robot.turret);
    }

    @Override
    public void initialize() {
        pidController.setSetPoint(setPoint);
        startTime = Calendar.getInstance().getTimeInMillis();
    }

    @Override
    public void execute() {
        robot.turret.setPower(pidController.calculate(robot.turret.getAngle()));
        FtcDashboard.getInstance().getTelemetry().addData("Turret is finished", isFinished());
    }

    @Override
    public boolean isFinished() {
        if(pidController.atSetPoint()) {
            return Calendar.getInstance().getTimeInMillis() - startTime > TIME_WAITING_FOR_TURRET_PID; //todo I can try to remove this and see if it goes faster. or that there's no difference, which means the problem is elsewhere.
        } else {
            startTime = Calendar.getInstance().getTimeInMillis();
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        robot.turret.stop();
    }

}
