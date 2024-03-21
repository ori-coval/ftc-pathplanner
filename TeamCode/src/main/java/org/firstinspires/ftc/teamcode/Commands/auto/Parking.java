package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryPoses;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class Parking extends ParallelCommandGroup {
    static RobotControl robot;
    public Parking(RobotControl robot) {
        Parking.robot = robot;
        addCommands(
                new ConditionalCommand(
                        getFarTrajectory(robot),
                        getCloseTrajectory(robot),
                        () -> robot.robotSide == AllianceSide.FAR
                ), //to allow arm to get back in
                new WaitCommand(500).andThen(new ArmGetToPosition(robot, ArmPosition.INTAKE, true))
        );
    }

    private Command getFarTrajectory(RobotControl robot) {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(TrajectoriesRed.FAR.trajectory, robot.autoDriveTrain),
                new TrajectoryFollowerCommand(TrajectoriesBlue.FAR.trajectory, robot.autoDriveTrain),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }


    private Command getCloseTrajectory(RobotControl robot) {
        return new ConditionalCommand(
                new ConditionalCommand(
                        new TrajectoryFollowerCommand(TrajectoriesRed.CLOSE_FRONT.trajectory, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(TrajectoriesRed.CLOSE.trajectory, robot.autoDriveTrain),
                        () ->  robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE
                ),
                new ConditionalCommand(
                        new TrajectoryFollowerCommand(TrajectoriesBlue.CLOSE_FRONT.trajectory, robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(TrajectoriesBlue.CLOSE.trajectory, robot.autoDriveTrain),
                        () ->  robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }


    public enum TrajectoriesRed {

        FAR(robot.autoDriveTrain.trajectorySequenceBuilder(ScoringCommand.TrajectoriesRed.CYCLES.trajectory.end())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        new Vector2d(-36, -50),
                        Math.toRadians(90) //Tangent
                )
                .build()
        ),

        CLOSE(robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.realBackdropClosePoseRed)
                .setTangent(180)
                .splineToConstantHeading(
                        new Vector2d(-60, -50),
                        Math.toRadians(90)
                )
                .build()
        ),

        CLOSE_FRONT(robot.autoDriveTrain.trajectorySequenceBuilder(ScoreYellowClose.TrajectoriesRed.CLOSE.trajectory.end())
                .splineToConstantHeading(
                        new Vector2d(-33, -50),
                        Math.toRadians(90)
                )
                .build()
        );

        final TrajectorySequence trajectory;

        TrajectoriesRed(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }

    }

    public enum TrajectoriesBlue {

        FAR(robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.realBackdropFarPoseBlue)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        new Vector2d(36, -50),
                        Math.toRadians(90) //Tangent
                )
                .build()
        ),

        CLOSE(robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.realBackdropClosePoseBlue)
                .setTangent(180)
                .splineToConstantHeading(
                        new Vector2d(60, -50),
                        Math.toRadians(90)
                )
                .build()
        ),

        CLOSE_FRONT(robot.autoDriveTrain.trajectorySequenceBuilder(ScoreYellowClose.TrajectoriesBlue.CLOSE.trajectory.end())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(
                        new Vector2d(33, -50),
                        Math.toRadians(90)
                )
                .build()
        );

        final TrajectorySequence trajectory;

        TrajectoriesBlue(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }
    }

}
