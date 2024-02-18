package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.BackToIntake;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.opencv.core.Mat;

import java.util.HashMap;

public class Trajectories {
    private static final HashMap<String , TrajectorySequence> trajectorySequenceHashMap = new HashMap<>();
    private static boolean isInitialized = false;

    public static void init(RobotControl robot, Pose2d startPose) {
        isInitialized = true; //initialized here in order to use the endPose

        //Purple Pixel Trajectories
        trajectorySequenceHashMap.put("Score Purple Right", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-50, 35), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-32, 26, Math.toRadians(45)), Math.toRadians(-45))
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Center", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-22, 35, Math.toRadians(45)))
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Left", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-23, 47, Math.toRadians(55)))
                .build()
        );

        //between Trajectories
        trajectorySequenceHashMap.put("loading intake right", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Right").end())
                .lineToLinearHeading(new Pose2d(-12, 47, Math.toRadians(90)))
                .build()
        );
        trajectorySequenceHashMap.put("loading intake center", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Center").end())
                .lineToLinearHeading(new Pose2d(-12, 47, Math.toRadians(90)))
                .build()
        );
        trajectorySequenceHashMap.put("loading intake left", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Left").end())
                .lineToLinearHeading(new Pose2d(-12, 47, Math.toRadians(90)))
                .build()
        );


        //Stack Trajectories
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Right", robot.autoDriveTrain.trajectorySequenceBuilder(get("loading intake right").end())
                .splineToLinearHeading(new Pose2d(-12, 55, Math.toRadians(90)), Math.toRadians(-90))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Center", robot.autoDriveTrain.trajectorySequenceBuilder(get("loading intake center").end())
                .splineToLinearHeading(new Pose2d(-12, 55, Math.toRadians(90)), Math.toRadians(-90))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Left", robot.autoDriveTrain.trajectorySequenceBuilder(get("loading intake left").end())
                .splineToLinearHeading(new Pose2d(-12, 55, Math.toRadians(90)), Math.toRadians(-90))
                .build()
        );

        trajectorySequenceHashMap.put("Drive back from stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack while avoiding pixel on Right").end())
                .back(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );
        trajectorySequenceHashMap.put("Drive back to stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Drive back from stack").end())
                .forward(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );

        //Scoring First
        trajectorySequenceHashMap.put("Go to backdrop part 1", robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.get("Driving to stack while avoiding pixel on Left").end())
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-12, -10, Math.toRadians(90)), Math.toRadians(-90))
                .build()
        );
        trajectorySequenceHashMap.put("Go to backdrop part 2", robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.get("Go to backdrop part 1").end())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-14, -63, Math.toRadians(90)), Math.toRadians(-90))
                .build()
        );


    }

    public static TrajectorySequence get(String trajectoryKey) {
        if (!isInitialized) {
            throw new RuntimeException("Trajectories aren't initialized.");
        }
        if (!trajectorySequenceHashMap.containsKey(trajectoryKey)) {
            throw new RuntimeException(trajectoryKey + " - Trajectory does not exist.");
        }
        return trajectorySequenceHashMap.get(trajectoryKey);
    }
}
