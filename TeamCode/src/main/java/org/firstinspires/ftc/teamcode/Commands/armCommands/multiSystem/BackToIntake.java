package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class BackToIntake extends SequentialCommandGroup {
    public BackToIntake(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret, Cartridge cartridge) {
        super(
                new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.INTAKE, false),
                new CartridgeSetState(cartridge, Cartridge.State.OPEN) //ready for next pixel
        );
    }
}
