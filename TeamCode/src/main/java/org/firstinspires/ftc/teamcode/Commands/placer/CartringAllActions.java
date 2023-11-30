package org.firstinspires.ftc.teamcode.Commands.placer;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

public class CartringAllActions extends CommandBase {
    private Cartridge cartridge;

    public CartringAllActions(Cartridge cartridge) {
        this.cartridge = cartridge;
        addRequirements(cartridge);


    }

    public void openCartidge() {
        cartridge.setState(Cartridge.State.OPEN);
    }

    public void close() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}








