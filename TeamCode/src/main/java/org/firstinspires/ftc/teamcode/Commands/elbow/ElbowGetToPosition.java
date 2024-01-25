package org.firstinspires.ftc.teamcode.Commands.elbow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Elbow;

public class ElbowGetToPosition extends CommandBase {
    private Elbow elbow;
    private double goalPos;
    private final double TOLERANCE = 0.2;
    //The arm moves using 2 axons, it can do short distances in no time. I'm only worried when the distance is large.

    public ElbowGetToPosition(Elbow elbow, double goalPos) {
        this.elbow = elbow;
        this.goalPos = goalPos;
    }

    @Override
    public void initialize() {
        elbow.setPosition(goalPos);
    }

    @Override
    public void execute() {
        FtcDashboard.getInstance().getTelemetry().addData("elbow is finished",isFinished());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elbow.getPosition() - goalPos)  < TOLERANCE;
    }
}
