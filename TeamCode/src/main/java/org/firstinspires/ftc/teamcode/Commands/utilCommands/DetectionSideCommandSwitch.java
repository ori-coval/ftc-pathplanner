package org.firstinspires.ftc.teamcode.Commands.utilCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SelectCommand;

import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

import java.util.HashMap;
import java.util.function.Supplier;

public class DetectionSideCommandSwitch extends SelectCommand {

    public DetectionSideCommandSwitch(Command far, Command center, Command close, Supplier<Object> sideSupplier) {
        super(new HashMap<Object, Command>() {
                  {
                      put(DetectionSide.FAR, far);
                      put(DetectionSide.CENTER, center);
                      put(DetectionSide.CLOSE, close);
                  }
              }, sideSupplier
        );

    }

}
