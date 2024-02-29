package org.firstinspires.ftc.teamcode.Commands.driveTrain;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotControl;

public class ResetFieldOriented extends InstantCommand {
    public ResetFieldOriented(RobotControl robot) {
        super(() -> robot.driveTrain.resetYaw());
    }
}
