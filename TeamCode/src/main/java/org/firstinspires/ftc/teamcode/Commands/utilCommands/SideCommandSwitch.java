package org.firstinspires.ftc.teamcode.Commands.utilCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SelectCommand;

import org.firstinspires.ftc.teamcode.Utils.Side;

import java.util.HashMap;
import java.util.function.Supplier;

public class SideCommandSwitch extends SelectCommand {
    public SideCommandSwitch(CommandBase left, CommandBase center, CommandBase right, Supplier<Object> sideSupplier) {
        super(new HashMap<Object, Command>() {
                  {
                      put(Side.LEFT, left);
                      put(Side.CENTER, center);
                      put(Side.RIGHT, right);
                  }
              }, sideSupplier
        );
    }
}
