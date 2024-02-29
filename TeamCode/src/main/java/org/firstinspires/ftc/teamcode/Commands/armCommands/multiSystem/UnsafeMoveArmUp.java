package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

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

public class UnsafeMoveArmUp extends SequentialCommandGroup {
    public UnsafeMoveArmUp(RobotControl robot, ArmPosition position, boolean isLeftOfBoard) {
        super(
                new ElbowGetToPosition(robot.elbow, position.getElbowPosition()),
                new ElevatorGetToHeightPID(robot.elevator, position.getElevatorHeight()),
                new RotateTurretByPID(robot, position.getTurretAngle(isLeftOfBoard)),
                new WaitCommand(UnsafeMoveArm.EXTENDER_WAIT_TIME), //Trying to avoid elbow's servos overload
                new ExtenderSetPosition(robot.extender, position.getExtenderPosition()),
                new AntiTurretGetToPosition(robot.antiTurret, position.getAntiTurretPosition())
        );
    }
}
