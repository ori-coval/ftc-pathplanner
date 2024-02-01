package org.firstinspires.ftc.teamcode.Commands.elbow;

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
    private final long TIME_BETWEEN_STEPS = 50; //(In ms)

    /*
    Still need to be tested, if too fast, you can always add more time between steps."
     */

    public ElbowGetToPosition(Elbow elbow, double goalPos) {
        this.elbow = elbow;
        this.goalPos = goalPos;
    }

    @Override
    public void initialize() {
        startTime = Calendar.getInstance().getTimeInMillis();
        startPos = elbow.getServoPosition();
    }

    @Override
    public void execute() {
        long timer = Calendar.getInstance().getTimeInMillis() - startTime;
        double stepSize = 0.05;
        double directionSign = Math.signum(goalPos - startPos);
        updateServoPosition(timer, stepSize, directionSign);

        FtcDashboard.getInstance().getTelemetry().addData("elbow is finished", isFinished());

    }

    private void updateServoPosition(long timer, double stepSize, double directionSign) {
        if (directionSign > 0) {
            elbow.setPosition(goalPos);
        } else if (timer > TIME_BETWEEN_STEPS * stepsTaken) {
            double newPos = startPos + directionSign * stepSize * stepsTaken; //if zero (though it should just call end (yeah directionSign can just be -1))
            if(newPos < goalPos) newPos = goalPos;
            elbow.setPosition(newPos);
            stepsTaken += 1;
        }
    }

    @Override
    public boolean isFinished() {
        return elbow.getServoPosition() == goalPos;
    }
}
