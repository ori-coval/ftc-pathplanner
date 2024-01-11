package org.firstinspires.ftc.teamcode.Commands.elbow;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.SubSystems.Elbow;

public class ElbowGetToAnglePID extends CommandBase {

    private Elbow elbow;
    private double goalAngle;
    private PIDController pidController;


    public ElbowGetToAnglePID(double goalAngle, Elbow elbow){
        this.elbow = elbow;
        this.goalAngle = goalAngle;
        addRequirements(elbow);
        pidController.setSetPoint(goalAngle);
        pidController.setTolerance(5);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return pidController.atSetPoint();
    }
}



