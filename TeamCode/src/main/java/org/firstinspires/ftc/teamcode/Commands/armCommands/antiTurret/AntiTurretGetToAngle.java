package org.firstinspires.ftc.teamcode.Commands.armCommands.antiTurret;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;

public class AntiTurretGetToAngle extends InstantCommand {
    public AntiTurretGetToAngle(AntiTurret antiTurret, double angle) {
        super(() -> antiTurret.setAngle(angle));
    }
}
