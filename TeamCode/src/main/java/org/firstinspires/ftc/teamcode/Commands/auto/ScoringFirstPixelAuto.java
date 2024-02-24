package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.inspection.InspectionState;

public class ScoringFirstPixelAuto extends SequentialCommandGroup {

    private final double RELEASE_PIXEL_HEIGHT = 28;

    public ScoringFirstPixelAuto(RobotControl robot){
        addCommands(
                new ParallelCommandGroup(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Go to backdrop"), robot.driveTrain),
                        new WaitCommand(1500).andThen(new SideCommandSwitch(
                                new InstantCommand(() -> robot.telemetry.addLine("Left")),
                                new InstantCommand(() -> robot.telemetry.addLine("Center")),
                                new InstantCommand(() -> robot.telemetry.addLine("Right")),

                                /*new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE, robot.allianceColor == AllianceColor.RED),
                                new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_MID, robot.allianceColor == AllianceColor.RED),
                                new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_FAR, robot.allianceColor == AllianceColor.RED),*/
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ).andThen(new InstantCommand(() -> robot.telemetry.update())))
                )/*,
                new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN),
                new ElevatorGetToHeightPID(robot.elevator, RELEASE_PIXEL_HEIGHT),
                new InstantCommand(() -> ArmGetToPosition.lastPosition = ArmPosition.INIFINITE_HEIGHT),
                new ArmGetToPosition(robot, ArmPosition.SCORING, robot.allianceColor == AllianceColor.RED),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED)*/
        );
    }

}
