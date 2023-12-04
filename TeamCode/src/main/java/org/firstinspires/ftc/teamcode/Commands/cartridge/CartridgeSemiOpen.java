package org.firstinspires.ftc.teamcode.Commands.cartridge;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

public class CartridgeSemiOpen extends CommandBase {
    private Cartridge cartridge;

    public CartridgeSemiOpen(Cartridge cartridge) {
        this.cartridge = cartridge;
        addRequirements(cartridge);
    }
    @Override
    public void initialize() {
        cartridge.setState(Cartridge.State.SEMI_OPEN);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}


