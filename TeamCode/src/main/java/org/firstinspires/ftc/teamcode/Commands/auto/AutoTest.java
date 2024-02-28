package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class AutoTest extends SequentialCommandGroup {
    public AutoTest(RobotControl robot) {
        super(
                new TrajectoryFollowerCommand(robot.trajectories.get("go to spike mark"), robot.autoDriveTrain)
        );
    }
}
