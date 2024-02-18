package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.utils.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

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
