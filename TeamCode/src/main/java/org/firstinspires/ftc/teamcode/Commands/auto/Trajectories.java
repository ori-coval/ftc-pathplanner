package org.firstinspires.ftc.teamcode.Commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.robot.Robot;

import org.apache.commons.math3.geometry.euclidean.twod.Segment;
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
        //Far
        //Left Detected
        trajectorySequenceHashMap.put("Far Purple Left", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 35, getY(47), getAngle(60)), getAngle(0))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 23, getY(47), getAngle(60)), getAngle(0))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack left", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple Left").end())
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, 47, getAngle(90)), getAngle(0))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 12, 55), getAngle(90))
                .build()
        );

        //Center Detected
        trajectorySequenceHashMap.put("Far Purple Center", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 40, getY(35), getAngle(60)), getAngle(0))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 22, getY(35), getAngle(45)), getAngle(0))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack center", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple Center").end())
                .setTangent(getAngle(45))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, 47, getAngle(90)), getAngle(90))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, 55, getAngle(90)), getAngle(90))
                .build()
        );

        //Right Detected
        trajectorySequenceHashMap.put("Far Purple Right", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 50, getY(38), getAngle(45)), getAngle(0))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 34, getY(26)), getAngle(-45))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack right", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple Right").end())
                .setTangent(getAngle(90))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, 47, getAngle(90)), getAngle(45))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 12, 55, getAngle(90)), getAngle(90))
                .build()
        );

        //Close
        //Left
        trajectorySequenceHashMap.put("Close Purple Left", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 40, -15, getAngle(-60)), getAngle(0))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 30, -5, getAngle(-60)), getAngle(90))
                .build()
        );

        //Center
        trajectorySequenceHashMap.put("Close Purple Center", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 40, -13, getAngle(-60)), getAngle(0))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 22, -13, getAngle(-45)), getAngle(0))
                .build()
        );

        //Right
        trajectorySequenceHashMap.put("Close Purple Right", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 50, -10, getAngle(-45)), getAngle(0))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 34, -22), getAngle(-45))
                .build()
        );


        //TODO remove later
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Right", robot.autoDriveTrain.trajectorySequenceBuilder(startPose).forward(2).build());
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Center", robot.autoDriveTrain.trajectorySequenceBuilder(startPose).forward(2).build());
        trajectorySequenceHashMap.put("Driving to stack while avoiding pixel on Left", robot.autoDriveTrain.trajectorySequenceBuilder(startPose).forward(2).build());
        trajectorySequenceHashMap.put("Go to backdrop part 1", robot.autoDriveTrain.trajectorySequenceBuilder(startPose).forward(2).build());
        trajectorySequenceHashMap.put("Go to backdrop part 2", robot.autoDriveTrain.trajectorySequenceBuilder(startPose).forward(2).build());



        trajectorySequenceHashMap.put("Drive back from stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack while avoiding pixel on Right").end())
                .back(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );
        trajectorySequenceHashMap.put("Drive back to stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Drive back from stack").end())
                .forward(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );

        //Scoring First
        trajectorySequenceHashMap.put("Go to backdrop", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack right").end())
                .setTangent(getAngle(-90))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, -10, getAngle(90)), getAngle(-90))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 7, -40, getAngle(90)), getAngle(-90))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 17, -64, getAngle(90)), getAngle(-180))
                .build()
        );

        trajectorySequenceHashMap.put("Close Yellow Right", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple Right").end())
                .setTangent(getAngle(180))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 60, -64, getAngle(90)), getAngle(-90))
                .build()
        );

        trajectorySequenceHashMap.put("Close Yellow Center", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple Center").end())
                .setTangent(getAngle(180))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 60, -64, getAngle(90)), getAngle(-90))
                .build()
        );

        trajectorySequenceHashMap.put("Close Yellow Left", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple Left").end())
                .setTangent(getAngle(-45))
                .splineToSplineHeading(new Pose2d(-7, -40, getAngle(-90)), getAngle(-90))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 15, -64), getAngle(180))
                .build()
        );


        //Parking
        trajectorySequenceHashMap.put("Backdrop Intake Far", robot.autoDriveTrain.trajectorySequenceBuilder(get("Go to backdrop").end())
                .setTangent(getAngle(90))
                .splineToConstantHeading(new Vector2d(-13, -51), getAngle(90))
                .build()
        );

        trajectorySequenceHashMap.put("Backdrop Intake Close Problematic", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Yellow Left").end())
                .setTangent(getAngle(90))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 15, -53), getAngle(90))
                .build()
        );

        trajectorySequenceHashMap.put("Backdrop Intake Close", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Yellow Right").end())
                .setTangent(getAngle(90))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 60, -53), getAngle(90))
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
