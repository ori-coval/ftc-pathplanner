package org.firstinspires.ftc.teamcode.Commands.armCommands.turret;

import org.firstinspires.ftc.teamcode.RobotControl;

public class TurretLeftJoystick extends TurretJoystick {
    private static final double TURRET_POWER = -0.5;
    public TurretLeftJoystick(RobotControl robot) {
        super(
                TURRET_POWER,
                robot
        );
    }
}
