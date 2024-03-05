package org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

public class CartridgeSetState extends ConditionalCommand {

    private static final long DELAY_BETWEEN_CARTRIDGE_STATE = 400;

    public CartridgeSetState(Cartridge cartridge, Cartridge.State state) {
        super(
                new SequentialCommandGroup(
                        instantCartridgeSetState(cartridge, Cartridge.State.SEMI_OPEN),
                        new WaitCommand(DELAY_BETWEEN_CARTRIDGE_STATE),
                        instantCartridgeSetState(cartridge, Cartridge.State.OPEN)
                ),
                instantCartridgeSetState(cartridge, state),
                () -> state == Cartridge.State.OPEN
        );
        addRequirements(cartridge);
    }

    private static Command instantCartridgeSetState(Cartridge cartridge, Cartridge.State state) {
        return new InstantCommand(() -> cartridge.setState(state));
    }

}
