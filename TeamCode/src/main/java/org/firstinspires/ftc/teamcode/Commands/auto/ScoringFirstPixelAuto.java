package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

public class ScoringFirstPixelAuto extends SequentialCommandGroup {

    private final double RELEASE_PIXEL_HEIGHT = 28;

    public ScoringFirstPixelAuto(RobotControl robot){
        addCommands(
                new TrajectoryFollowerCommand(Trajectories.get("Go to backdrop part 1"), robot.driveTrain),
                new SideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE, true),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_MID, true),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_FAR, true),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new TrajectoryFollowerCommand(Trajectories.get("Go to backdrop part 2"), robot.driveTrain),
                new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN),
                new ElevatorGetToHeightPID(robot.elevator, RELEASE_PIXEL_HEIGHT),
                new InstantCommand(() -> ArmGetToPosition.lastPosition = ArmPosition.INIFINITE_HEIGHT),
                new ArmGetToPosition(robot, ArmPosition.SCORING, true),
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED)
        );
    }

}
