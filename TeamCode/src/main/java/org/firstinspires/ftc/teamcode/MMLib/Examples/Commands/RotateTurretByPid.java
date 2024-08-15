package org.firstinspires.ftc.teamcode.MMLib.Examples.Commands;

import org.firstinspires.ftc.teamcode.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

public class RotateTurretByPid extends MMPIDCommand {
    public RotateTurretByPid(double setPoint) {
        super(MMRobot.getInstance().mmSystems.shooterTurret, setPoint);
    }
}
