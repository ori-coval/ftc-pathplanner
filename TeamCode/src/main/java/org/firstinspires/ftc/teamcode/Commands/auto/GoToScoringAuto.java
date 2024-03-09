package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class GoToScoringAuto extends ParallelCommandGroup {
    public GoToScoringAuto(RobotControl robot) {
        super(
                new ArmGetToPosition(robot, ArmPosition.SCORING_AUTO, robot.allianceColor == AllianceColor.RED),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
        );
    }
}
