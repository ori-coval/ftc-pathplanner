package org.firstinspires.ftc.teamcode.Commands.turret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class GetMaxMin extends CommandBase {
    double min;
    double max;

    Turret turret;
    public GetMaxMin(Turret turret, double min, double max){
        this.min = min;
        this.max = max;
        this.turret = turret;
        addRequirements(turret);
    }

}
