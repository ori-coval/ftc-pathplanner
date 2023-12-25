package org.firstinspires.ftc.teamcode.Commands.multisystem;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
    private Elevator elevator;
    private Extender extender;
    private Elbow elbow;
    private Turret turret;
    private final double ANGLE_THRESHOLD = 0;

    private boolean armCanRotate() {
        final double minCartridgeHeightToMoveArm = 0;
        double axisDistFromFloor = elevator.TOP_DIST_FROM_FLOOR + elevator.getHeight();
        double l = extender.getLength();
        double CartridgeHeight = axisDistFromFloor - l * Math.cos(Math.toRadians(elbow.getAngle()));
        return CartridgeHeight > minCartridgeHeightToMoveArm;
    }

    private boolean armInCenter() {
        return (turret.getAngle() < ANGLE_THRESHOLD) && (turret.getAngle() > -ANGLE_THRESHOLD);
    }

    public ArmGetToPosition(Elevator elevator, Elbow elbow, Extender extender, Turret turret, ArmPosition position, boolean isLeftOfBoard) {
        this.elevator = elevator;
        this.extender = extender;
        this.elbow = elbow;
        this.turret = turret;
        addCommands(
                new WaitUntilCommand(this::armInCenter).andThen(
                        new ParallelCommandGroup(
                                new ElevatorGetToHeightPID(elevator, position.getElevatorHeight()),
                                new ElbowGetToAnglePID(elbow, position.getElbowAngle())
                        )
                ),
                new WaitUntilCommand(this::armCanRotate).andThen(
                        new ParallelCommandGroup(
                                new ExtenderSetLength(extender, position.getExtenderLength()),
                                new RotateTurretByPID(turret, position.getTurretAngle(isLeftOfBoard))
                        )
                )
        );
    }
}
