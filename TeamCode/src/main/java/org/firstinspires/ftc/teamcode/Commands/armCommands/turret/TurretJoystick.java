package org.firstinspires.ftc.teamcode.Commands.armCommands.turret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Commands.armCommands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;

public class TurretJoystick extends CommandBase {

    private final double ELBOW_TOLERANCE = 0.1;
    private final double ELBOW_SAFE_POS = 0.9;
    RobotControl robot;
    double power;

    public TurretJoystick(double power, RobotControl robot) {
        this.power = power;
        this.robot = robot;
    }

    @Override
    public void initialize() {
        if(robot.inDebugMode) {
            if(Math.abs(robot.elbow.getServoPosition() - ELBOW_SAFE_POS) > ELBOW_TOLERANCE) {
                new ElbowGetToPosition(robot.elbow, ELBOW_SAFE_POS).schedule(); //todo waitCommand after, so the arm wouldn't break or smth
            }
            robot.turret.setPower(power);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(robot.inDebugMode) {
            robot.turret.stop();
        }
    }
}
