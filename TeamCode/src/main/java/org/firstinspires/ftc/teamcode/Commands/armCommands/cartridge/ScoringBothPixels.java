package org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.ElevatorGetToHeightPID;
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
                new InstantCommand(() -> robot.elevator.setPower(0.5)),
                new WaitCommand(500),
                new InstantCommand(() -> robot.elevator.setPower(0)),
                new SideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORING, true),
                        new ArmGetToPosition(robot, ArmPosition.SAFE_PLACE, false),
                        new ArmGetToPosition(robot, ArmPosition.SCORING, false),
                        ArmPositionSelector::getSelectedRobotSide
                ),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED)
        );
    }
}
