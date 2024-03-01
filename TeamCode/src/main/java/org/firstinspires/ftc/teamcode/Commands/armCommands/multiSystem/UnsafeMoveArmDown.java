package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.antiTurret.AntiTurretGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.armCommands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class UnsafeMoveArmDown extends SequentialCommandGroup {
    public UnsafeMoveArmDown(RobotControl robot, ArmPosition position, boolean isLeftOfBoard) {
        super(
                new RotateTurretByPID(robot, position.getTurretAngle(isLeftOfBoard)),
//                new WaitCommand(UnsafeMoveArm.ELEVATOR_WAIT_TIME), //avoiding elevator's shaking while going down too fast.
                new ElevatorGetToHeightPID(robot, position.getElevatorHeight()),
                new AntiTurretGetToPosition(robot.antiTurret, position.getAntiTurretPosition()),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ElbowGetToPosition(robot.elbow, position.getElbowPosition()),
                                new WaitCommand(UnsafeMoveArm.EXTENDER_WAIT_TIME),
                                /*Todo this might not be needed due to the fact that the elbow has a isFinished method, so it is waiting anyway.*/
                                new ExtenderSetPosition(robot.extender, position.getExtenderPosition())
                        ),
                        new SequentialCommandGroup(
                                new ExtenderSetPosition(robot.extender, position.getExtenderPosition()),
                                new WaitCommand(UnsafeMoveArm.EXTENDER_WAIT_TIME), //TODO this also might be a problem, because we are talking about 3 different pos, which means 2 calls for this command, which means a whole 1/2 second which is being waited here. try to reduce and check.
                                new ElbowGetToPosition(robot.elbow, position.getElbowPosition())
                        ),
                        () -> (ArmGetToPosition.lastPosition.getExtenderPosition().getServoPositionAsDouble() < position.getExtenderPosition().getServoPositionAsDouble())
                )
        );
    }
}
