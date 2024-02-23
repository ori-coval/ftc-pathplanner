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
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.opencv.core.Mat;

import java.util.HashMap;

public class Trajectories {
    private final HashMap<String , TrajectorySequence> trajectorySequenceHashMap = new HashMap<>();
    private final RobotControl robot;
    double trajectorySignAlliance;


    private double getAngle(double angle) {
        double result;
        if(robot.allianceColor == AllianceColor.RED) {
            result = Math.toRadians(angle);
        } else {
            result = Math.toRadians(180 - angle);
        }
        return result;
    }

    private double getY(double y) {
        return robot.robotSide == AllianceSide.FAR ? y : y - 48;
    }

    public Trajectories(RobotControl robot, Pose2d startPose) {
        this.robot = robot;

        trajectorySignAlliance = ((robot.allianceColor == AllianceColor.BLUE) ? 1 : (-1));

        //Purple Pixel Trajectories
        //Left Detected
        trajectorySequenceHashMap.put("Score Purple Left", robot.driveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 35, getY(47), getAngle(60)), getAngle(0))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 23, getY(47), getAngle(60)), getAngle(0))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack left", robot.driveTrain.trajectorySequenceBuilder(get("Score Purple Left").end())
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, 47, getAngle(90)), getAngle(0))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 12, 55), getAngle(90))
                .build()
        );

        //Center Detected
        trajectorySequenceHashMap.put("Score Purple Center", robot.driveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 40, getY(35), getAngle(60)), getAngle(0))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 22, getY(35), getAngle(45)), getAngle(0))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack center", robot.driveTrain.trajectorySequenceBuilder(get("Score Purple Center").end())
                .setTangent(getAngle(45))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, 47, getAngle(90)), getAngle(90))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, 55, getAngle(90)), getAngle(90))
                .build()
        );

        //Right Detected
        trajectorySequenceHashMap.put("Score Purple Right", robot.driveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 50, getY(38), getAngle(45)), getAngle(0))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 34, getY(26)), getAngle(-45))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack right", robot.driveTrain.trajectorySequenceBuilder(get("Score Purple Right").end())
                .setTangent(getAngle(90))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, 47, getAngle(90)), getAngle(45))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 13, 55, getAngle(90)), getAngle(90))
                .build()
        );


        //TODO remove later
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Right", robot.driveTrain.trajectorySequenceBuilder(startPose).forward(2).build());
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Center", robot.driveTrain.trajectorySequenceBuilder(startPose).forward(2).build());
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Left", robot.driveTrain.trajectorySequenceBuilder(startPose).forward(2).build());
        trajectorySequenceHashMap.put("Go to backdrop part 1", robot.driveTrain.trajectorySequenceBuilder(startPose).forward(2).build());
        trajectorySequenceHashMap.put("Go to backdrop part 2", robot.driveTrain.trajectorySequenceBuilder(startPose).forward(2).build());



        trajectorySequenceHashMap.put("Drive back from stack", robot.driveTrain.trajectorySequenceBuilder(get("Driving to stack while avoiding pixel on Right").end())
                .back(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );
        trajectorySequenceHashMap.put("Drive back to stack", robot.driveTrain.trajectorySequenceBuilder(get("Drive back from stack").end())
                .forward(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );

        //Scoring First
        trajectorySequenceHashMap.put("Go to backdrop", robot.driveTrain.trajectorySequenceBuilder(get("Driving to stack right").end())
                .setTangent(getAngle(-90))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, -10, getAngle(90)), getAngle(-90))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 7, -40, getAngle(90)), getAngle(-90))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 13.5, -63, getAngle(90)), getAngle(-180))
                .build()
        );

        trajectorySequenceHashMap.put("Close Yellow Right", robot.driveTrain.trajectorySequenceBuilder(get("Score Purple Right").end())
                .setTangent(getAngle(180))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 60, -60, getAngle(-90)), getAngle(-90))
                .build()
        );


        //Parking
        trajectorySequenceHashMap.put("Go back after scoring yellow", robot.driveTrain.trajectorySequenceBuilder(get("Go to backdrop").end())
                .setTangent(getAngle(90))
                .splineToConstantHeading(new Vector2d(-13, -51), getAngle(90))
                .build()
        );

        /*
        //Purple Pixel Trajectories
        trajectorySequenceHashMap.put("Score Purple Right", robot.driveTrain.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 50, getY(isSideLeft, 35)), getAngle(robot, 0))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 32, getY(isSideLeft, 26), getAngle(robot, 45)), getAngle(robot, -45))
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Center", robot.driveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(trajectorySignAlliance * 22, getY(isSideLeft, 35), getAngle(robot, 45)))
                .build()
        );
        trajectorySequenceHashMap.put("Score Purple Left", robot.driveTrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(trajectorySignAlliance * 23, getY(isSideLeft, 47), getAngle(robot, 55)))
                .build()
        );

        //between Trajectories
        Pose2d beforeStack = new Pose2d(trajectorySignAlliance * 12, 47, getAngle(robot, 90));
        trajectorySequenceHashMap.put("loading intake right", robot.driveTrain.trajectorySequenceBuilder(get("Score Purple Right").end())
                .lineToLinearHeading(beforeStack)
                .build()
        );
        trajectorySequenceHashMap.put("loading intake center", robot.driveTrain.trajectorySequenceBuilder(get("Score Purple Center").end())
                .lineToLinearHeading(beforeStack)
                .build()
        );
        trajectorySequenceHashMap.put("loading intake left", robot.driveTrain.trajectorySequenceBuilder(get("Score Purple Left").end())
                .lineToLinearHeading(beforeStack)
                .build()
        );


        //Stack Trajectories
        Pose2d stackPose = new Pose2d(trajectorySignAlliance * 12, 55, getAngle(robot, 90));
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Right", robot.driveTrain.trajectorySequenceBuilder(get("loading intake right").end())
                .splineToLinearHeading(stackPose, getAngle(robot, -90))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Center", robot.driveTrain.trajectorySequenceBuilder(get("loading intake center").end())
                .splineToLinearHeading(stackPose, getAngle(robot, -90))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Left", robot.driveTrain.trajectorySequenceBuilder(get("loading intake left").end())
                .splineToLinearHeading(stackPose, getAngle(robot, -90))
                .build()
        );


        MecanumVelocityConstraint velocityDown = new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.3, DriveConstants.TRACK_WIDTH);
        ProfileAccelerationConstraint accelerationDown = new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.3);
        trajectorySequenceHashMap.put("Drive back from stack", robot.driveTrain.trajectorySequenceBuilder(get("Driving to stack while avoiding pixel on Right").end())
                .back(8, velocityDown, accelerationDown)
                .build()
        );
        trajectorySequenceHashMap.put("Drive back to stack", robot.driveTrain.trajectorySequenceBuilder(get("Drive back from stack").end())
                .forward(8, velocityDown, accelerationDown)
                .build()
        );*/



        //Parking Right
        Pose2d parkingPose = new Pose2d(trajectorySignAlliance * 60, -63, getAngle(-90));
        trajectorySequenceHashMap.put("Parking right (left)", robot.driveTrain.trajectorySequenceBuilder(get("Score Purple Left").end())
                .splineToSplineHeading(parkingPose, getAngle(-90))
                .build()
        );

        trajectorySequenceHashMap.put("Parking right (center)", robot.driveTrain.trajectorySequenceBuilder(get("Score Purple Center").end())
                .splineToSplineHeading(parkingPose, getAngle(-90))
                .build()
        );

        trajectorySequenceHashMap.put("Parking right (right)", robot.driveTrain.trajectorySequenceBuilder(get("Score Purple Right").end())
                .splineToSplineHeading(parkingPose, getAngle(-90))
                .build()
        );


    }

    public TrajectorySequence get(String trajectoryKey) {
        if (!trajectorySequenceHashMap.containsKey(trajectoryKey)) {
            throw new RuntimeException(trajectoryKey + " - Trajectory does not exist.");
        }
        return trajectorySequenceHashMap.get(trajectoryKey);
    }
}
