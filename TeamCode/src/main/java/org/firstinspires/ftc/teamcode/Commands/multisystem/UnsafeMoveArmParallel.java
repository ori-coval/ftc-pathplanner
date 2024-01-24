package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.antiTurret.AntiTurretGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class UnsafeMoveArmParallel extends ParallelCommandGroup {
    private Elevator elevator;
    private Extender extender;
    private Elbow elbow;
    private Turret turret;
    private AntiTurret antiTurret;

    public UnsafeMoveArmParallel (Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret, ArmPosition position, boolean isLeftOfBoard) {
        this.elevator = elevator;
        this.extender = extender;
        this.elbow = elbow;
        this.turret = turret;
        this.antiTurret = antiTurret;
        addCommands(
                new ElevatorGetToHeightPID(elevator, position.getElevatorHeight()),
                new RotateTurretByPID(turret, position.getTurretAngle(isLeftOfBoard)),
                new ElbowGetToPosition(elbow, position.getElbowAngle()),
                new ExtenderSetPosition(extender, position.getExtenderPosition()),
                new AntiTurretGetToPosition(antiTurret, position.getAntiTurretAngle())
        );
    }
}
