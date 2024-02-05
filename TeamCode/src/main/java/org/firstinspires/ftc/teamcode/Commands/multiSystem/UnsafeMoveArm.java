package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

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

public class UnsafeMoveArm extends ParallelCommandGroup {
    public UnsafeMoveArm(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret, ArmPosition position, boolean isLeftOfBoard) {
        addCommands(
                new ElbowGetToPosition(elbow, position.getElbowPosition()).andThen(new ParallelCommandGroup(
                            new ElevatorGetToHeightPID(elevator, position.getElevatorHeight()),
                            new RotateTurretByPID(turret, position.getTurretAngle(isLeftOfBoard))
                        )).andThen(new ExtenderSetPosition(extender, position.getExtenderPosition())),
                new AntiTurretGetToPosition(antiTurret, position.getAntiTurretPosition())
        );
    }

    @Override
    public void execute() {
        super.execute();
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
