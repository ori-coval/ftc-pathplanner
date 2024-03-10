package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.ResetPixelCount;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

import java.util.function.Supplier;

public class ScoringCommand extends SequentialCommandGroup {
    public ScoringCommand(Command scoringCommand, Command secondScoringCommand, RobotControl robot, Double numOfCycle) {
        addCommands(
                new ParallelCommandGroup(
                        getTrajectoryCommand(robot),
                        new IntakeRotate(robot.intake.roller, robot.intake.roller.EJECT_POWER).withTimeout(1500),
                        new WaitCommand(1700).andThen(
                                new ArmGetToPosition(robot, ArmPosition.SCORING, robot.allianceColor == AllianceColor.RED),
                                new InstantCommand(() -> RotateTurretByPID.DEADLINE_FOR_TURRET = 700),//todo maybe will work with less time
                                scoringCommand
                        )
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new CartridgeSetState(robot.cartridge, Cartridge.State.SEMI_OPEN),
                                new WaitCommand(200),
                                new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN)
                        ),
                        new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN),
                        () -> numOfCycle == 1
                ),
                new ResetPixelCount(robot),
                new InstantCommand(() -> RotateTurretByPID.DEADLINE_FOR_TURRET = 2000),
                new WaitCommand(300)
// <- todo remove this when we need elevator               secondScoringCommand
        );
    }


    private Command getTrajectoryCommand(RobotControl robot) {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(robot.trajectories.get("Go to backdrop (Far Side) Red"), robot.autoDriveTrain).andThen(resetPoseEstimate(robot)),
                new TrajectoryFollowerCommand(robot.trajectories.get("Go to backdrop (Far Side) Blue"), robot.autoDriveTrain).andThen(resetPoseEstimate(robot)),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    private Command resetPoseEstimate(RobotControl robot) {
        return new ConditionalCommand(
                new InstantCommand(() ->
                        robot.autoDriveTrain.setPoseEstimate(new Pose2d(
                                robot.trajectories.trajectoryPoses.realBackdropPoseRed.getX(),
                                robot.trajectories.trajectoryPoses.realBackdropPoseRed.getY(),
                                robot.autoDriveTrain.getPoseEstimate().getHeading()
                        ))
                ),
                new InstantCommand(() ->
                    robot.autoDriveTrain.setPoseEstimate(new Pose2d(
                            robot.trajectories.trajectoryPoses.realBackdropPoseBlue.getX(),
                            robot.trajectories.trajectoryPoses.realBackdropPoseBlue.getY(),
                            robot.autoDriveTrain.getPoseEstimate().getHeading()
                    ))
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

}
