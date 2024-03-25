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

public class ScoringTeleCommand extends SequentialCommandGroup {
    public ScoringTeleCommand(RobotControl robot, BooleanSupplier triggerCondition, Cartridge.State cartridgePosition) {
        super(
                new CartridgeSetState(robot.cartridge, cartridgePosition),
                new WaitUntilCommand(() -> !triggerCondition.getAsBoolean()),
                new SideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORING, true),
                        new ArmGetToPosition(robot, ArmPosition.SAFE_PLACE, true),
                        new ArmGetToPosition(robot, ArmPosition.SCORING, false),
                        ArmPositionSelector::getSelectedRobotSide
                ),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
        );
        addRequirements(robot.cartridge, robot.turret, robot.elevator, robot.elbow, robot.extender, robot.antiTurret);
    }
}
