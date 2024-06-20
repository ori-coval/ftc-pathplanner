package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.MMLib.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

public class RotateTurretByPid extends MMPIDCommand {
    public RotateTurretByPid(double setPoint) {
        super(MMRobot.getInstance().mmSystems.shooterTurret, setPoint);
    }
}
