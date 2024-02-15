package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.HashMap;

public class Trajectories {
    private static final HashMap<String , TrajectorySequence> trajectorySequenceHashMap = new HashMap<>();
    private static boolean isInitialized = false;
    public static void init(SampleMecanumDrive driveTrain){
        trajectorySequenceHashMap.put("zigzug testing", driveTrain.trajectorySequenceBuilder(new Pose2d(-63, 35))
                .splineTo(new Vector2d(-44, 20), 0)
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
