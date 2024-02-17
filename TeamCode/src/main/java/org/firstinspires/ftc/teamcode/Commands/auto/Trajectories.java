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
import org.opencv.core.Mat;

import java.util.HashMap;

public class Trajectories {
    private static final HashMap<String , TrajectorySequence> trajectorySequenceHashMap = new HashMap<>();
    private static boolean isInitialized = false;

    public static TrajectoryVelocityConstraint trajectoryVelocityConstraint;
    public static void init(SampleMecanumDrive driveTrain, Pose2d startPose, RobotControl robot) {
        isInitialized = true; //initialized here in order to use the endPose

        //Purple Pixel Trajectories
        trajectorySequenceHashMap.put("Score Purple Right", driveTrain.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-50, 35), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-32, 26, Math.toRadians(45)), Math.toRadians(-45))
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Center", driveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-22, 35, Math.toRadians(45)))
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Left", driveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-23, 47, Math.toRadians(70)))
                .build()
        );


        //Stack Trajectories
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Left", driveTrain.trajectorySequenceBuilder(get("Score Purple Left").end())
                .splineToLinearHeading(new Pose2d(-12, 55, Math.toRadians(90)), Math.toRadians(90))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Center", driveTrain.trajectorySequenceBuilder(get("Score Purple Center").end())
                .splineToLinearHeading(new Pose2d(-12, 55, Math.toRadians(90)), Math.toRadians(90))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Right", driveTrain.trajectorySequenceBuilder(get("Score Purple Right").end())
                .setTangent(Math.toRadians(110))
                .addDisplacementMarker(3, () -> {
                    new BackToIntake(robot.elevator, robot.elbow, robot.extender, robot.turret, robot.antiTurret, robot.cartridge).schedule();
                })
                .splineToLinearHeading(new Pose2d(-12, 56, Math.toRadians(90)), Math.toRadians(90))
                .build()
        );
        trajectorySequenceHashMap.put("Drive back from stack", driveTrain.trajectorySequenceBuilder(get("Driving to stack while avoiding pixel on Right").end())
                .back(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );
        trajectorySequenceHashMap.put("Drive back to stack", driveTrain.trajectorySequenceBuilder(get("Drive back from stack").end())
                .forward(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );

        //Scoring First
        trajectorySequenceHashMap.put("Go to backdrop", driveTrain.trajectorySequenceBuilder(get("Drive back from stack").end())
                .splineToSplineHeading(new Pose2d(-12, -10, Math.toRadians(90)), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    new ArmGetToPosition(robot.elevator, robot.elbow, robot.extender, robot.turret, robot.antiTurret, ArmPosition.SCORING, true);
                })
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
