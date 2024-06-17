package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.MMLib.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

public class ShootByPID extends MMPIDCommand {
    public ShootByPID(double setPoint) {
        super(MMRobot.getInstance().mmSystems.shooterPID, setPoint);
    }
}
