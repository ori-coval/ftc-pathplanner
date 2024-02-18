package org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

public class CartridgeSetState extends InstantCommand {

    public CartridgeSetState(Cartridge cartridge, Cartridge.State state){
        super(()-> cartridge.setState(state), cartridge);
    }

}
