package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

public class ElevatorGetToPosition extends MMPIDCommand {
    public ElevatorGetToPosition(double setPoint) {
        super(MMRobot.getInstance().mmSystems.elevator, setPoint);
    }

    @Override
    public void execute() {
        super.execute();
        FtcDashboard.getInstance().getTelemetry().addData("is finished: ", false);
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        FtcDashboard.getInstance().getTelemetry().addData("is finished",true);
        FtcDashboard.getInstance().getTelemetry().update();

    }
}
