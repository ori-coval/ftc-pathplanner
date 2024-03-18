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
                new TrajectoryFollowerCommand(robot.trajectories.get("Backdrop Intake Close (Close Detected)"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(robot.trajectories.get("Backdrop Intake Close"), robot.autoDriveTrain),
                () ->  robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE
        );
    }


    public enum TrajectoriesRed {

        FAR(robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.realBackdropPoseRed)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(-10, -57),
                        Math.toRadians(30) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(-13, -51),
                        Math.toRadians(180), //Tangent
                        robot.trajectories.reduceVelocity(0.7),
                        robot.trajectories.reduceAcceleration(0.7)
                )
                .build()
        );

        final TrajectorySequence trajectory;

        TrajectoriesRed(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }

    }

    public enum TrajectoriesBlue {

        FAR(robot.autoDriveTrain.trajectorySequenceBuilder(TrajectoryPoses.realBackdropPoseBlue)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(
                        new Vector2d(10, -57),
                        Math.toRadians(90) //Tangent
                )
                .splineToConstantHeading(
                        new Vector2d(13, -51),
                        Math.toRadians(45), //Tangent
                        robot.trajectories.reduceVelocity(0.7),
                        robot.trajectories.reduceAcceleration(0.7)
                )
                .build()
        );

        final TrajectorySequence trajectory;

        TrajectoriesBlue(TrajectorySequence trajectory) {
            this.trajectory = trajectory;
        }
    }

}
