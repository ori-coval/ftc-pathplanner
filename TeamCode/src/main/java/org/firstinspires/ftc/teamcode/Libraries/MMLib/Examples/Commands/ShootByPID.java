package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Commands;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

/**
 * notice that there's literally nothing here.
 * u could really just use the {@link MMPIDCommand} with 1 more parameter to insert.
 * u should inherit if u were planning on overriding the logic of the init, execute, isFinished or end methods.
 */
public class ShootByPID extends MMPIDCommand {
    public ShootByPID(double setPoint) {
        super(MMRobot.getInstance().mmSystems.shooterPID, setPoint);
    }
}
