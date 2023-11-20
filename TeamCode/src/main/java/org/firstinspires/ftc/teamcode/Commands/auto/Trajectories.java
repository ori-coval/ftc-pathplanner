package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.HashMap;

public class Trajectories {
    private static HashMap<String , TrajectorySequence> trajectorySequenceHashMap;
    private static boolean isInitialized = false;
    public static void init(SampleMecanumDrive driveTrain){
        trajectorySequenceHashMap.put("3", driveTrain.trajectorySequenceBuilder(new Pose2d()).build());
        trajectorySequenceHashMap.put("2", driveTrain.trajectorySequenceBuilder(new Pose2d()).build());
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
