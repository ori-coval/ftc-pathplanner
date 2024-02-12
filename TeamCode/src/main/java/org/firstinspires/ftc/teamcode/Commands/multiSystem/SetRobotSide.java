package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.Side;

import java.util.HashMap;

public class SetRobotSide extends SelectCommand {
    public SetRobotSide(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret, Side side) {
        super(
                new HashMap<Object, Command>() {{
                    put(Side.LEFT, new SequentialCommandGroup(
                        
                    ));
                    put(Side.CENTER, new SequentialCommandGroup(

                    ));
                    put(Side.RIGHT, new SequentialCommandGroup(

                    ));
                }}, () -> side
        );
    }

}
