package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.antiTurret.AntiTurretGetToAngle;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.armCommands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.RobotControl;

public class UnsafeMoveArmUp extends SequentialCommandGroup {
    public UnsafeMoveArmUp(RobotControl robot, ArmPosition position, boolean isLeftOfBoard) {
        super(
                new ParallelCommandGroup(
                        new ElbowGetToPosition(robot.elbow, position.getElbowPosition()),
                        new ElevatorGetToHeightPID(robot, position.getElevatorHeight())
                ),
                new AntiTurretGetToAngle(robot.antiTurret, position.getAntiTurretAngle(isLeftOfBoard)),
                new ExtenderSetPosition(robot.extender, position.getExtenderPosition()),
                new WaitCommand(200),
                new RotateTurretByPID(robot, position.getTurretAngle(isLeftOfBoard))
        );
    }
}
