package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.arcrobotics.ftclib.command.CommandBase;
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

    ArmGetToPosition command;

    public ArmGetToSelectedPosition(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret){
        this.elevator = elevator;
        this.elbow = elbow;
        this.extender = extender;
        this.turret = turret;
        this.antiTurret = antiTurret;
    }
    @Override
    public void initialize() {
        command = new ArmGetToPosition(elevator, elbow, extender,turret,antiTurret,ArmPositionSelector.getPosition(), ArmPositionSelector.getIsLeftOfBoard());
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
