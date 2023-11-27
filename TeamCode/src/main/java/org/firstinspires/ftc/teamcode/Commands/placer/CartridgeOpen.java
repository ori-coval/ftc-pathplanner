package org.firstinspires.ftc.teamcode.Commands.placer;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

public class CartridgeOpen extends CommandBase {

    private Cartridge cartridge;

    public CartridgeOpen(Cartridge cartridge) {
        this.cartridge = cartridge;
        addRequirements(cartridge);
    }

    @Override
    public void initialize() {
        cartridge.setState(Cartridge.State.OPEN);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
