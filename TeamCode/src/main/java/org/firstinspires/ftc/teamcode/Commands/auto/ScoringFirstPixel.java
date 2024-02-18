package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

public class ScoringFirstPixel extends SequentialCommandGroup {
    public ScoringFirstPixel(RobotControl robot){
        super(
                new TrajectoryFollowerCommand(Trajectories.get("Go to backdrop part 1"), robot.autoDriveTrain),
                new ArmGetToPosition(robot, ArmPosition.SCORING, true),
                new TrajectoryFollowerCommand(Trajectories.get("Go to backdrop part 2"), robot.autoDriveTrain),
                new SideCommandSwitch(
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE, true),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_FAR, true),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_MID_FAR, true),
                        () -> robot.teamPropDetector.getTeamPropSide()),
                new CartridgeSetState(robot.cartridge,Cartridge.State.OPEN)
        );
    }

}
