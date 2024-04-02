package org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;

public class Trajectories {

    public static Pose2d realBackdropFarPoseRed = new Pose2d(-17.7165, -62.15748, Math.toRadians(90));
    public static Pose2d realBackdropClosePoseRed = new Pose2d(0, 0, 0); //todo need to find this
    public static Pose2d realBackdropFarPoseBlue = new Pose2d(17.7, -64.9, Math.toRadians(90));
    public static Pose2d realBackdropClosePoseBlue = new Pose2d(0, 0, 0); //todo need to find this
    public static Pose2d realBackdropFront = new Pose2d(0, -49.9527559, 0);
    public static Pose2d stackPoseRed = new Pose2d(-13, 59, Math.toRadians(90));
    public static Pose2d stackPoseBlue = new Pose2d(16, 59, Math.toRadians(90));

    public static MecanumVelocityConstraint reduceVelocity(double newVelocity) {
        return new MecanumVelocityConstraint(DriveConstants.MAX_VEL * newVelocity, DriveConstants.TRACK_WIDTH);
    }

    public static ProfileAccelerationConstraint reduceAcceleration(double newAcceleration) {
        return new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * newAcceleration);
    }

}
