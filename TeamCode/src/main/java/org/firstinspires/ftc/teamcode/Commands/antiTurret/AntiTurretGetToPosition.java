package org.firstinspires.ftc.teamcode.Commands.antiTurret;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;

import java.util.function.DoubleSupplier;

public class AntiTurretGetToPosition extends InstantCommand {
    public AntiTurretGetToPosition(AntiTurret antiTurret, double position) {
        super(() -> antiTurret.setPos(position));
    }
}
