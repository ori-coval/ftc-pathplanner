package org.firstinspires.ftc.teamcode.Commands.multisystem;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.elbow.ElbowGetToAnglePID;
import org.firstinspires.ftc.teamcode.Commands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.extender.ExtenderSetLength;
import org.firstinspires.ftc.teamcode.Commands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ArmGetToPosition extends ParallelCommandGroup {
    Elevator elevator;
    Extender extender;
    Elbow elbow;
    private boolean canArmMove(){
        final double minCartridgeHeightToMoveArm = 0;
        double axisDistFromFloor = elevator.TOP_DIST_FROM_FLOOR + elevator.getHeight();
        double l = extender.getLength();
        double CartridgeHeight = axisDistFromFloor - l*Math.cos(Math.toRadians(elbow.getAngle()));
        return CartridgeHeight > minCartridgeHeightToMoveArm;
    }
    public ArmGetToPosition(Elevator elevator, Elbow elbow, Extender extender, Turret turret, ArmPosition position, boolean isLeftOfBoard){
        this.elevator = elevator;
        this.extender = extender;

        addCommands(
                new ElevatorGetToHeightPID(elevator, position.getElevatorHeight()),
                new ElbowGetToAnglePID(elbow, position.getElbowAngle()),
                new SequentialCommandGroup(
                        new WaitUntilCommand(()-> canArmMove()),
                        new ParallelCommandGroup(
                                new ExtenderSetLength(extender, position.getExtenderLength()),
                                new RotateTurretByPID(turret, position.getTurretAngle(isLeftOfBoard))
                        )
                )
        );
    }
}
