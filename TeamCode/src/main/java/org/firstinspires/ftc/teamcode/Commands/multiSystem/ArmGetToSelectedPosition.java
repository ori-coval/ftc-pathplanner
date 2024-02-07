package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.ProxyScheduleCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;

import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ArmGetToSelectedPosition extends CommandBase {

    Elevator elevator;
    Elbow elbow;
    Extender extender;
    Turret turret;
    AntiTurret antiTurret;

    Command command;

    public ArmGetToSelectedPosition(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret){
        this.elevator = elevator;
        this.elbow = elbow;
        this.extender = extender;
        this.turret = turret;
        this.antiTurret = antiTurret;
    }
    @Override
    public void initialize() {
        command = new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPositionSelector.getPosition(), ArmPositionSelector.getIsLeftOfBoard());
        FtcDashboard.getInstance().getTelemetry().addData("POS",ArmPositionSelector.getPosition() + ", isLeftOfBoard " + ArmPositionSelector.getIsLeftOfBoard());
        FtcDashboard.getInstance().getTelemetry().update();
        command.schedule();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        command.cancel();
        command = new PerpetualCommand(new InstantCommand());
        FtcDashboard.getInstance().getTelemetry().addData("ended", ArmPositionSelector.getPosition());
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

}
