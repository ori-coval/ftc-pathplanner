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

        Pose2d stackPos = new Pose2d(trajectorySignAlliance * 11.5, 57, getAngle(90));

        //Purple Pixel Trajectories
        //Robot Far From Backdrop
        //Far From Truss Detected
        trajectorySequenceHashMap.put("Far Purple (Far Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 35, getY(47), getAngle(60)), getAngle(0))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 23, getY(47), getAngle(60)), getAngle(0))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack (Far Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Far Detected)").end())
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, 47, getAngle(90)), getAngle(0))
                .splineToConstantHeading(new Vector2d(stackPos.getX(), stackPos.getY()), getAngle(90))
                .build()
        );

        //Center Detected
        trajectorySequenceHashMap.put("Far Purple (Center Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 40, getY(35), getAngle(60)), getAngle(0))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 22, getY(35), getAngle(45)), getAngle(0))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack (Center Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Center Detected)").end())
                .setTangent(getAngle(45))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, 47, getAngle(90)), getAngle(90))
                .splineToSplineHeading(stackPos, getAngle(90))
                .build()
        );

        //Close To Truss Detected
        trajectorySequenceHashMap.put("Far Purple (Close Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 50, getY(38), getAngle(45)), getAngle(0))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 34, getY(26)), getAngle(-45))
                .build()
        );
        trajectorySequenceHashMap.put("Driving to stack (Close Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Far Purple (Close Detected)").end())
                .setTangent(getAngle(45))
                .splineToLinearHeading(stackPos, getAngle(45))
                .build()
        );

        //Robot Close To Backdrop Side
        //Close to truss
        trajectorySequenceHashMap.put("Close Purple (Close Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 40, -15, getAngle(-60)), getAngle(0))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 30, -5, getAngle(-60)), getAngle(90))
                .build()
        );

        trajectorySequenceHashMap.put("Close Yellow (Close Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple (Close Detected)").end())
                .setTangent(getAngle(-45))
                .splineToSplineHeading(new Pose2d(-7, -40, getAngle(-90)), getAngle(-90))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 15, -64), getAngle(180))
                .build()
        );

        //Center
        trajectorySequenceHashMap.put("Close Purple (Center Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 40, -13, getAngle(-60)), getAngle(0))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 22, -13, getAngle(-45)), getAngle(0))
                .build()
        );

        trajectorySequenceHashMap.put("Close Yellow (Center Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple (Center Detected)").end())
                .setTangent(getAngle(180))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 60, -64, getAngle(90)), getAngle(-90))
                .build()
        );

        //Far from truss
        trajectorySequenceHashMap.put("Close Purple (Far Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 50, -10, getAngle(-45)), getAngle(0))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 34, -22), getAngle(-45))
                .build()
        );
        
        trajectorySequenceHashMap.put("Close Yellow (Far Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Purple (Far Detected)").end())
                .setTangent(getAngle(180))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 60, -64, getAngle(90)), getAngle(-90))
                .build()
        );


        //Bits
        trajectorySequenceHashMap.put("Drive back from stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack (Close Detected)").end())
                .back(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );
        trajectorySequenceHashMap.put("Drive back to stack", robot.autoDriveTrain.trajectorySequenceBuilder(get("Drive back from stack").end())
                .forward(8, new MecanumVelocityConstraint(DriveConstants.MAX_VEL * 0.2, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.2))
                .build()
        );

        //Scoring Far
        trajectorySequenceHashMap.put("Go to backdrop", robot.autoDriveTrain.trajectorySequenceBuilder(get("Driving to stack (Close Detected)").end())
                .setTangent(getAngle(-90))
                .splineToSplineHeading(new Pose2d(trajectorySignAlliance * 12, -10, getAngle(90)), getAngle(-90))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 7, -40, getAngle(90)), getAngle(-90))
                .splineToLinearHeading(new Pose2d(trajectorySignAlliance * 17, -64, getAngle(90)), getAngle(-180))
                .build()
        );


        //Parking
        trajectorySequenceHashMap.put("Parking Arm To Intake (Far Side)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Go to backdrop").end())
                .setTangent(getAngle(90))
                .splineToConstantHeading(new Vector2d(-13, -51), getAngle(90))
                .build()
        );

        trajectorySequenceHashMap.put("Backdrop Intake Close (Close Detected)", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Yellow (Close Detected)").end())
                .setTangent(getAngle(90))
                .splineToConstantHeading(new Vector2d(trajectorySignAlliance * 15, -53), getAngle(90))
                .build()
        );

        trajectorySequenceHashMap.put("Backdrop Intake Close", robot.autoDriveTrain.trajectorySequenceBuilder(get("Close Yellow (Far Detected)").end())
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
