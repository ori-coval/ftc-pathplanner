package org.firstinspires.ftc.teamcode.Commands.elbow;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Elbow;

public class ElbowGetToAngle extends CommandBase {
    private double goalAngle;
    private Elbow elbow;
    public ElbowGetToAngle(Elbow elbow, double goalAngle){
        this.goalAngle = goalAngle;
        this.elbow = elbow;
        this.addRequirements(elbow);
    }
    private boolean directionUp;

    @Override
    public void initialize() {
        double currentAngle = elbow.getAngle();
        double deltaAngle = goalAngle - currentAngle;
        directionUp = (deltaAngle >= 0);
    }


    @Override
    public void execute() {
        elbow.setPower((directionUp? 1:-1) * 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        elbow.stop();
    }

    @Override
    public boolean isFinished() {
        if (directionUp){
            return goalAngle < elbow.getAngle();
        }
        return goalAngle > elbow.getAngle();

    }
}
