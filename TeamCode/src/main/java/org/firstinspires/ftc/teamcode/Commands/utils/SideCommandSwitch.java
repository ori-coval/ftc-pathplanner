package org.firstinspires.ftc.teamcode.Commands.utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SelectCommand;

import org.firstinspires.ftc.teamcode.Commands.auto.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Side;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SideCommandSwitch extends SelectCommand {

    public SideCommandSwitch(CommandBase left, CommandBase center, CommandBase right, Supplier<Object> sideSupplier) {
        super(new HashMap<Object, Command>() {
                  {
                      put(Side.RIGHT, right);
                      put(Side.CENTER, center);
                      put(Side.LEFT, left);
                  }
              }, sideSupplier
        );

    }

}
