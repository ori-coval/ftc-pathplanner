package org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

import java.util.function.BooleanSupplier;

public class ScoringFirstPixel extends SequentialCommandGroup {
    public ScoringFirstPixel(Cartridge cartridge, BooleanSupplier triggerCondition) {
        super(
                new CartridgeSetState(cartridge, Cartridge.State.SEMI_OPEN),
                new WaitUntilCommand(() -> !triggerCondition.getAsBoolean()),
                new CartridgeSetState(cartridge, Cartridge.State.CLOSED)
        );
    }
}
