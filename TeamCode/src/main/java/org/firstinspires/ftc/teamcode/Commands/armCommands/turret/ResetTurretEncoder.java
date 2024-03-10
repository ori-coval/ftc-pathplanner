package org.firstinspires.ftc.teamcode.Commands.armCommands.turret;

import org.firstinspires.ftc.teamcode.Commands.utilCommands.OnlyIfDebugMode;
import org.firstinspires.ftc.teamcode.RobotControl;

public class ResetTurretEncoder extends OnlyIfDebugMode {
    public ResetTurretEncoder(RobotControl robot) {
        super(
                () -> robot.turret.resetEncoder(),
                robot
        );
    }
}
