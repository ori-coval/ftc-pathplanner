package org.firstinspires.ftc.teamcode.Commands.utilCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SelectCommand;

import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

import java.util.HashMap;
import java.util.function.Supplier;

public class ScoringByDetectionSwitch extends ConditionalCommand {
    public ScoringByDetectionSwitch(CommandBase far, CommandBase center, CommandBase close, Supplier<Object> sideSupplier, RobotControl robot) {
        super(
                new SelectCommand(new HashMap<Object, Command>() {
                    {
                        put(DetectionSide.FAR, far);
                        put(DetectionSide.CENTER, center);
                        put(DetectionSide.CLOSE, center);
                    }
                }, sideSupplier),
                new SelectCommand(new HashMap<Object, Command>() {
                    {
                        put(DetectionSide.FAR, far);
                        put(DetectionSide.CENTER, center);
                        put(DetectionSide.CLOSE, close);
                    }
                }, sideSupplier),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }
}
