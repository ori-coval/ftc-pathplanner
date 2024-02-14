package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.Side;

import java.util.HashMap;

public class SetRobotSide extends SequentialCommandGroup {
    public SetRobotSide(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret, Side side) {
        super(
                new InstantCommand(() -> ArmPositionSelector.setRobotSide(side)),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(Side.LEFT, new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORING, true));
                            put(Side.CENTER, new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SAFE_PLACE, false));
                            put(Side.RIGHT, new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORING, false));
                        }}, () -> side
                )
        );
    }

}