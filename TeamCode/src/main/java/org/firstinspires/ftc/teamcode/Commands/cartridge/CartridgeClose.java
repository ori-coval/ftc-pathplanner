package org.firstinspires.ftc.teamcode.Commands.cartridge;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

public class CartridgeClose extends CommandBase {
    private Cartridge cartridge;

    public CartridgeClose(Cartridge cartridge) {
        this.cartridge = cartridge;
        addRequirements(cartridge);
    }

    @Override
    public void initialize() {
        cartridge.setState(Cartridge.State.CLOSED);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
