package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.antiTurret.AntiTurretGetToAngle;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.armCommands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.RobotControl;

public class UnsafeMoveArmDown extends SequentialCommandGroup {
    public UnsafeMoveArmDown(RobotControl robot, ArmPosition position, boolean isLeftOfBoard) {
        super(
                new ParallelCommandGroup(
                        new ExtenderSetPosition(robot.extender, position.getExtenderPosition()),
                        new RotateTurretByPID(robot, position.getTurretAngle(isLeftOfBoard)),
                        new AntiTurretGetToAngle(robot.antiTurret, position.getAntiTurretAngle(isLeftOfBoard))
                ),
                new ElevatorGetToHeightPID(robot, position.getElevatorHeight()),
                new ElbowGetToPosition(robot.elbow, position.getElbowPosition())
                /*new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ElbowGetToPosition(robot.elbow, position.getElbowPosition()),
                                new ExtenderSetPosition(robot.extender, position.getExtenderPosition()),
                        ),
                        new SequentialCommandGroup(
                                new ExtenderSetPosition(robot.extender, position.getExtenderPosition()), //Todo if the extender still makes problems with the turret. bring this to the top of the command, and hope for the elbow not to break. *skull emoji*
                                new WaitCommand(UnsafeMoveArm.EXTENDER_WAIT_TIME),
                                new ElbowGetToPosition(robot.elbow, position.getElbowPosition())
                        ),
                        () -> (ArmGetToPosition.lastPosition.getExtenderPosition().getServoPositionAsDouble() < position.getExtenderPosition().getServoPositionAsDouble())
                )*/
        );
    }
}
