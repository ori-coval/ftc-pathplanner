package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Commands;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

public class ShootByPID extends MMPIDCommand {
    public ShootByPID(double setPoint) {
        super(MMRobot.getInstance().mmSystems.shooterPID, setPoint);
    }
}
