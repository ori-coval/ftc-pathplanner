package org.firstinspires.ftc.teamcode.Commands.elbow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Elbow;

import java.util.Calendar;

public class ElbowGetToPosition extends CommandBase {
    private Elbow elbow;
    private double goalPos;
    private final double TOLERANCE = 0.05;
    //The arm moves using 2 axons, it can do short distances in no time. I'm only worried when the distance is large.

    private long startTime;
    private double startPos;
    private int stepsTaken;


    public ElbowGetToPosition(Elbow elbow, double goalPos) {
        this.elbow = elbow;
        this.goalPos = goalPos;
    }

    @Override
    public void initialize() {
        startTime = Calendar.getInstance().getTimeInMillis();
        startPos = elbow.getServoPosition();
        stepsTaken = 1;
    }

    @Override
    public void execute() {
        long timer = Calendar.getInstance().getTimeInMillis() - startTime;
        double stepSize = 0.05;
        double isDirectionPositive = Math.signum(goalPos - elbow.getEncoderPosition());
        if (isDirectionPositive > 0) {
            elbow.setPosition(goalPos);
        } else if (timer > (long) 50 * stepsTaken) {
            elbow.setPosition(startPos + isDirectionPositive * stepSize * stepsTaken);
            stepsTaken += 1;
        }

        FtcDashboard.getInstance().getTelemetry().addData("elbow is finished", isFinished());
        FtcDashboard.getInstance().getTelemetry().addData("elbow error ", Math.abs(elbow.getEncoderPosition() - goalPos));

    }

    @Override
    public boolean isFinished() {
        return Math.abs(elbow.getEncoderPosition() - goalPos) < TOLERANCE;
    }
}
