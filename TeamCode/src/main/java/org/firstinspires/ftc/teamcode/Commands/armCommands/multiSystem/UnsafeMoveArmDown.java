package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.antiTurret.AntiTurretGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.armCommands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class UnsafeMoveArmDown extends SequentialCommandGroup {
    public UnsafeMoveArmDown(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret, ArmPosition position, boolean isLeftOfBoard) {
        super(
                new RotateTurretByPID(turret, position.getTurretAngle(isLeftOfBoard)),
                new WaitCommand(UnsafeMoveArm.ELEVATOR_WAIT_TIME), //avoiding elevator's shaking while going down too fast.
                new ElevatorGetToHeightPID(elevator, position.getElevatorHeight()),
                new ExtenderSetPosition(extender, position.getExtenderPosition()),
                new ElbowGetToPosition(elbow, position.getElbowPosition()),
                new AntiTurretGetToPosition(antiTurret, position.getAntiTurretPosition())
        );
    }
}

//אם האקסטנדר נסגר אז קודם אקסטנדר ואז אלבוו, אם אקסטנדר נפתח אז קודם אלבוו ואז אקסטנדר
