package org.firstinspires.ftc.teamcode.Commands.armCommands.elbow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Elbow;

import java.util.Calendar;

public class ElbowGetToPosition extends CommandBase {
    private Elbow elbow;
    private double goalPos;
    private long startTime;
    private double startPos;
    private int stepsTaken = 1;
    private final double STEP_SIZE = 0.05;
    private final long TIME_BETWEEN_STEPS = 50; //(In ms)
    private final double TOLERANCE = 0.05;
    /*
    If it's too fast, you can always add more time between steps.
     */

    public ElbowGetToPosition(Elbow elbow, double goalPos) {
        this.elbow = elbow;
        this.goalPos = goalPos;
        elbow.setPosition(0);
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
        double directionSign = Math.signum(goalPos - startPos);
        updateServoPosition(timer, directionSign);

        FtcDashboard.getInstance().getTelemetry().addData("elbow is finished", isFinished());

    }

    private void updateServoPosition(long timer, double directionSign) {
        FtcDashboard.getInstance().getTelemetry().addLine("in UpdateServoPosition" + stepsTaken);
            if (directionSign > 0) {
                elbow.setPosition(goalPos);
            } else if (timer > TIME_BETWEEN_STEPS * stepsTaken) {
                double newPos = startPos - STEP_SIZE * stepsTaken;
                if(newPos < goalPos) newPos = goalPos;
                elbow.setPosition(newPos);
                FtcDashboard.getInstance().getTelemetry().addData("elbow's goalPos", goalPos);
                FtcDashboard.getInstance().getTelemetry().addData("elbow's currentPos", elbow.getServoPosition());
                FtcDashboard.getInstance().getTelemetry().addData("elbow's newPos", newPos);
                stepsTaken += 1;
            }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elbow.getServoPosition() - goalPos) < TOLERANCE;
    }
}
