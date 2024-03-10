package org.firstinspires.ftc.teamcode.Commands.armCommands.turret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Commands.armCommands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;

public class TurretRightJoystick extends CommandBase {

    RobotControl robot;

    public TurretRightJoystick(RobotControl robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        if(robot.inDebugMode) {
            new ElbowGetToPosition(robot.elbow, 0.9).schedule();
            robot.turret.setPower(0.3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(robot.inDebugMode) {
            robot.turret.stop();
        }
    }
}
