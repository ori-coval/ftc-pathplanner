package org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

import java.util.function.BooleanSupplier;

public class ScoringBothPixels extends SequentialCommandGroup {
    public ScoringBothPixels(RobotControl robot, BooleanSupplier triggerCondition) {
        super(
                new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN),
                new WaitUntilCommand(() -> !triggerCondition.getAsBoolean()),
                new SideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORING, true),
                        new ArmGetToPosition(robot, ArmPosition.SAFE_PLACE, false),
                        new ArmGetToPosition(robot, ArmPosition.SCORING, false),
                        ArmPositionSelector::getSelectedRobotSide
                ),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
        );
    }
}
