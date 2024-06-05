package org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;

public class Trajectories {

    public static Pose2d pose1 = new Pose2d(0, 0, 0); //todo need to find this
    public static Pose2d pose2 = new Pose2d(0, 0, 0); //todo need to find this

    public static MecanumVelocityConstraint reduceVelocity(double newVelocity) {
        return new MecanumVelocityConstraint(DriveConstants.MAX_VEL * newVelocity, DriveConstants.TRACK_WIDTH);
    }

    public static ProfileAccelerationConstraint reduceAcceleration(double newAcceleration) {
        return new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL * newAcceleration);
    }

}
