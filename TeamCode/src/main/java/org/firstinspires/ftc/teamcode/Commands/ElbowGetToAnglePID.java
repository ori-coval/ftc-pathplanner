package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.SubSystems.Elbow;

public class ElbowGetToAnglePID extends CommandBase {

    Elbow elbow;
    PIDController pid = new PIDController(0,0,0);
    double goalAngle;


    public ElbowGetToAnglePID(double goalAngle, Elbow elbow){
        this.elbow = elbow;
        this.goalAngle = goalAngle;
        addRequirements(elbow);
        pid.setSetPoint(goalAngle);
        pid.setTolerance(5);
    }

    @Override
    public void execute() {
        elbow.setPower(pid.calculate(elbow.getAngle()));
    }

    @Override
    public void end(boolean interrupted) {
        elbow.stop();
    }

    @Override
    public boolean isFinished() {
        return pid.atSetPoint();
    }
}



