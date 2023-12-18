package org.firstinspires.ftc.teamcode.Commands.multisystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.elbow.ElbowGetToAnglePID;
import org.firstinspires.ftc.teamcode.Commands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.extender.ExtenderSetLength;
import org.firstinspires.ftc.teamcode.Commands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ArmGetToPosition extends SequentialCommandGroup {
    Elevator elevator;
    Extender extender;
    private boolean canArmMove(){
        private final double minElevatorHeightToOpen = 0
        double axisDistFromFloor = elevator.TOP_DIST_FROM_FLOOR + elevator.getHeight();
        double l = 0
        //todo find l using cosine rule, change extender to cm, angle and all that
        double pointHeight = axisDistFromFloor - l*Math.cos();
        return pointHeight > minElevatorHeightToOpen;
    }
    public ArmGetToPosition(Elevator elevator, Elbow elbow, Extender extender, Turret turret, ArmPosition position, boolean isLeftOfBoard){
        this.elevator = elevator;
        this.extender = extender;
        addCommands(
                new ElevatorGetToHeightPID(elevator, position.getElevatorHeight()),
                new ElbowGetToAnglePID(elbow, position.getElbowAngle()),
                new ExtenderSetLength(extender, position.getExtenderLength()),
                new RotateTurretByPID(turret, position.getTurretAngle(isLeftOfBoard))
        );
    }
}
