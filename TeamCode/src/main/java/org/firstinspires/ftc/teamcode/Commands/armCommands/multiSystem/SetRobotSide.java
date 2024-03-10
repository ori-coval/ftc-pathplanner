package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.Utils.Side;

public class SetRobotSide extends SequentialCommandGroup {
    public SetRobotSide(RobotControl robot, Side side) {
        super(
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS),
                new InstantCommand(() -> ArmPositionSelector.setRobotSide(side)),
                new SideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORING, true),
                        new ArmGetToPosition(robot, ArmPosition.SAFE_PLACE, true),
                        new ArmGetToPosition(robot, ArmPosition.SCORING, false),
                        () -> side
                )
        );
    }

}
