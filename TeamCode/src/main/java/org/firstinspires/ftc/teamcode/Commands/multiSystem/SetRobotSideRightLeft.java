package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class SetRobotSideRightLeft extends CommandBase {
    private Elevator elevator;
    private Elbow elbow;
    private Extender extender;
    private Turret turret;
    private AntiTurret antiTurret;
    private Side side;
    private SequentialCommandGroup command;

    public SetRobotSideRightLeft(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret, Side side) {
        this.elevator = elevator;
        this.elbow = elbow;
        this.extender = extender;
        this.turret = turret;
        this.antiTurret = antiTurret;
        this.side = side;
    }

    @Override
    public void initialize() {
        command = new SequentialCommandGroup(
                new InstantCommand(() -> ArmPositionSelector.setRobotSide(side)),
                new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORING, ArmPositionSelector.getIsLeftOfBoard())
        );
        command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        command.cancel();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
