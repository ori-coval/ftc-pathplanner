package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.Side;

import java.util.HashMap;

public class SetRobotSide extends SequentialCommandGroup {
    public SetRobotSide(RobotControl robot, Side side) {
        super(
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED),
                new InstantCommand(() -> ArmPositionSelector.setRobotSide(side)),
                new SideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORING, true),
                        new ArmGetToPosition(robot, ArmPosition.SAFE_PLACE, false),
                        new ArmGetToPosition(robot, ArmPosition.SCORING, false),
                        () -> side
                )
        );
    }

}
