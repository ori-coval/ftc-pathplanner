package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.robot.Robot;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.BackToIntake;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.opencv.core.Mat;

import java.util.HashMap;

public class Trajectories {
    private static final HashMap<String , TrajectorySequence> trajectorySequenceHashMap = new HashMap<>();
    private static boolean isInitialized = false;

    private static double getAngle(RobotControl robot, double angle) {
        double result;
        if(robot.allianceColor == AllianceColor.RED) {
            result = Math.toRadians(angle);
        } else {
            result = Math.toRadians(180 - angle);
        }
        if(robot.robotSide == Side.RIGHT) {
            result = -result;
        }
        return result;
    }

    private static int getY(boolean isSideLeft, int y) {
        return isSideLeft ? y : y - 48;
    }

    public static void init(RobotControl robot, Pose2d startPose) {
        isInitialized = true; //initialized here in order to use the endPose

        double trajectorySignAlliance = ((robot.allianceColor == AllianceColor.BLUE) ? 1 : (-1));
        boolean isSideLeft = robot.robotSide == Side.LEFT;


        trajectorySequenceHashMap.put("Score Purple Center RightBlue", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(40, 35, Math.toRadians(-100)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(20, 35, Math.toRadians(180)), Math.toRadians(180))
                .build()
        );

        trajectorySequenceHashMap.put("Score Purple Center LeftBlue", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(40, -11, Math.toRadians(-100)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(20, -11, Math.toRadians(180)), Math.toRadians(180))
                .build()
        );

        trajectorySequenceHashMap.put("Park on LeftBlue", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Center LeftBlue").end())
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(10, -63), Math.toRadians(-90))
                .build()
        );

        trajectorySequenceHashMap.put("Park on rightBlue", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Center LeftBlue").end())
                .build()
        );


        //Purple Pixel Trajectories
        trajectorySequenceHashMap.put("Score Purple Left", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-35, 47, Math.toRadians(60)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-23, 47, Math.toRadians(60)), Math.toRadians(0))
                .build()
        );
        trajectorySequenceHashMap.put("loading intake left", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Left").end())
                .splineToSplineHeading(new Pose2d(-12, 47, Math.toRadians(90)), Math.toRadians(0))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Left", robot.autoDriveTrain.trajectorySequenceBuilder(get("loading intake left").end())
                .splineToConstantHeading(new Vector2d(-12, 55), Math.toRadians(90))
                .build()
        );
/*        trajectorySequenceHashMap.put("Score Purple Left (Blue)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(35, 47, Math.toRadians(120)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(23, 47, Math.toRadians(120)), Math.toRadians(180))
                .build()
        );
        trajectorySequenceHashMap.put("loading intake left (Blue)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Left").end())
                .splineToSplineHeading(new Pose2d(12, 47, Math.toRadians(90)), Math.toRadians(180))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Left (Blue)", robot.autoDriveTrain.trajectorySequenceBuilder(get("loading intake left").end())
                .splineToConstantHeading(new Vector2d(12, 55), Math.toRadians(90))
                .build()
        );*/
        trajectorySequenceHashMap.put("Score Purple Center", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-40, 35, Math.toRadians(60)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-22, 35, Math.toRadians(45)), Math.toRadians(0))
                .build()
        );
        trajectorySequenceHashMap.put("loading intake center", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Center").end())
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-12, 47, Math.toRadians(90)), Math.toRadians(90))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Center", robot.autoDriveTrain.trajectorySequenceBuilder(get("loading intake center").end())
                .splineToSplineHeading(new Pose2d(-12, 55, Math.toRadians(90)), Math.toRadians(90))
                .build()
        );

        trajectorySequenceHashMap.put("Score Purple Right", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-50, 35), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-34, 26, Math.toRadians(45)), Math.toRadians(-45))
                .build()
        );
        trajectorySequenceHashMap.put("loading intake right", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Right").end())
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-22, 35, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-12, 47, Math.toRadians(90)), Math.toRadians(90))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Right", robot.autoDriveTrain.trajectorySequenceBuilder(get("loading intake right").end())
                .splineToSplineHeading(new Pose2d(-13, 58, Math.toRadians(90)), Math.toRadians(90))
                .build()
        );

        trajectorySequenceHashMap.put("Score Purple Center Right Blue", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(40, 35, Math.toRadians(120)), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(22, 35, Math.toRadians(135)), Math.toRadians(180))
                .build()
        );


        //between Trajectories


        //Stack Trajectories




        trajectorySequenceHashMap.put("Drive back from stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack while avoiding pixel on Right").end())
                .back(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );
        trajectorySequenceHashMap.put("Drive back to stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Drive back from stack").end())
                .forward(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );

        //Scoring First
        trajectorySequenceHashMap.put("Go to backdrop part 1", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack while avoiding pixel on Left").end())
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-12, -10, Math.toRadians(90)), Math.toRadians(-90))
                .build()
        );
        trajectorySequenceHashMap.put("Go to backdrop part 2", robot.autoDriveTrain.trajectorySequenceBuilder(get("Go to backdrop part 1").end())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-7, -40, Math.toRadians(90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-13.5, -63, Math.toRadians(90)), Math.toRadians(-180))
                .build()
        );


        /*
        //Purple Pixel Trajectories
        trajectorySequenceHashMap.put("Score Purple Right", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 50, getY(isSideLeft, 35)), getAngle(robot, 0))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 32, getY(isSideLeft, 26), getAngle(robot, 45)), getAngle(robot, -45))
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Center", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(trajectorySignAlliance * 22, getY(isSideLeft, 35), getAngle(robot, 45)))
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Left", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(trajectorySignAlliance * 23, getY(isSideLeft, 47), getAngle(robot, 55)))
                .build()
        );

        //between Trajectories
        Pose2d beforeStack = new Pose2d(trajectorySignAlliance * 12, 47, getAngle(robot, 90));
        trajectorySequenceHashMap.put("loading intake right", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Right").end())
                .lineToLinearHeading(beforeStack)
                .build()
        );
        trajectorySequenceHashMap.put("loading intake center", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Center").end())
                .lineToLinearHeading(beforeStack)
                .build()
        );
        trajectorySequenceHashMap.put("loading intake left", robot.autoDriveTrain.trajectorySequenceBuilder(get("Score Purple Left").end())
                .lineToLinearHeading(beforeStack)
                .build()
        );


        //Stack Trajectories
        Pose2d stackPose = new Pose2d(trajectorySignAlliance * 12, 55, getAngle(robot, 90));
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Right", robot.autoDriveTrain.trajectorySequenceBuilder(get("loading intake right").end())
                .splineToLinearHeading(stackPose, getAngle(robot, -90))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Center", robot.autoDriveTrain.trajectorySequenceBuilder(get("loading intake center").end())
                .splineToLinearHeading(stackPose, getAngle(robot, -90))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Left", robot.autoDriveTrain.trajectorySequenceBuilder(get("loading intake left").end())
                .splineToLinearHeading(stackPose, getAngle(robot, -90))
                .build()
        );


        MecanumVelocityConstraint velocityDown = new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.3, DriveConstants.TRACK_WIDTH);
        ProfileAccelerationConstraint accelerationDown = new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.3);
        trajectorySequenceHashMap.put("Drive back from stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack while avoiding pixel on Right").end())
                .back(8, velocityDown, accelerationDown)
                .build()
        );
        trajectorySequenceHashMap.put("Drive back to stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Drive back from stack").end())
                .forward(8, velocityDown, accelerationDown)
                .build()
        );

        //Scoring First
        trajectorySequenceHashMap.put("Go to backdrop part 1", robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.get("Driving to stack while avoiding pixel on Left").end())
                .setTangent(getAngle(robot, -90))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, -10, getAngle(robot, 90)), getAngle(robot, -90))
                .build()
        );
        trajectorySequenceHashMap.put("Go to backdrop part 2", robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.get("Go to backdrop part 1").end())
                .setTangent(getAngle(robot, -90))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 13, -63, getAngle(robot, 90)), getAngle(robot, -90))
                .build()
        );*/

        trajectorySequenceHashMap.put("Go back after scoring yellow", robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.get("Go to backdrop part 2").end())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-13, -51), Math.toRadians(90))
                .build()
        );


        //Parking Right
        Pose2d parkingPose = new Pose2d(trajectorySignAlliance * 60, -63, getAngle(robot, -90));
        trajectorySequenceHashMap.put("Parking right (left)", robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.get("Score Purple Left").end())
                .splineToSplineHeading(parkingPose, getAngle(robot, -90))
                .build()
        );

        trajectorySequenceHashMap.put("Parking right (center)", robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.get("Score Purple Center").end())
                .splineToSplineHeading(parkingPose, getAngle(robot, -90))
                .build()
        );

        trajectorySequenceHashMap.put("Parking right (right)", robot.autoDriveTrain.trajectorySequenceBuilder(Trajectories.get("Score Purple Right").end())
                .splineToSplineHeading(parkingPose, getAngle(robot, -90))
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
