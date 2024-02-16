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
        isInitialized = true; //initialized here in order to use the endPose

        //Purple Pixel Trajectories
        trajectorySequenceHashMap.put("Go to middle before scoring purple pixel on Right", driveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-37, 35, Math.toRadians(45)))
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Right", driveTrain.trajectorySequenceBuilder(get("Go to middle before scoring purple pixel on Right").end())
                .strafeTo(new Vector2d(-37, 26))
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Center", driveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(get("Go to middle before scoring purple pixel on Right").end())
                .strafeTo(new Vector2d(-22, 35))
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Left", driveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(get("Go to middle before scoring purple pixel on Right").end())
                .strafeTo(new Vector2d(-34, 52))
                .build()
        );


        //Stack Trajectories
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Left", driveTrain.trajectorySequenceBuilder(get("Score Purple Left").end())
                .lineToLinearHeading(new Pose2d(-24, 60, Math.toRadians(90)))
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
