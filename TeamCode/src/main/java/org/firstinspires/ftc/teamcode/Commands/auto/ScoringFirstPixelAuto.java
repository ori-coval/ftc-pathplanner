package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class ScoringFirstPixelAuto extends SequentialCommandGroup {
    public ScoringFirstPixelAuto(RobotControl robot){
        super(
                new ParallelCommandGroup(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Go to backdrop"), robot.autoDriveTrain),
                        new WaitCommand(1500).andThen(new DetectionSideCommandSwitch(
                                new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE, robot.allianceColor == AllianceColor.RED),
                                new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_MID, robot.allianceColor == AllianceColor.RED),
                                new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_FAR, robot.allianceColor == AllianceColor.RED),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        )
                ),
                new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN),
                new ArmGetToPosition(robot, ArmPosition.SCORING, robot.allianceColor == AllianceColor.RED),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED)
                )
        );
    }

}
