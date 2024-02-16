package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.HashMap;

public class Trajectories {
    private static final HashMap<String , TrajectorySequence> trajectorySequenceHashMap = new HashMap<>();
    private static boolean isInitialized = false;
    public static void init(SampleMecanumDrive driveTrain, Pose2d startPose) {
        trajectorySequenceHashMap.put("Go to middle before scoring purple pixel", driveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-37, 35, Math.toRadians(45))) //-22
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Pixel Center", driveTrain.trajectorySequenceBuilder(new Pose2d(-37, 35, Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(-22, 35, Math.toRadians(45)))
                .build()
        );
        isInitialized = true;
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
