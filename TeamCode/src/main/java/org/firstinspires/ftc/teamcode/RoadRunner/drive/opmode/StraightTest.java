package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

/**
 * This is a simple routine to test translational drive capabilities.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends CommandOpMode {

    public static double DISTANCE = 60; // in

    private DriveTrain drive;
    private TrajectoryFollowerCommand straightFollower;

    @Override
    public void initialize() {
        drive = new DriveTrain(new SampleMecanumDrive(hardwareMap), false);
        straightFollower = new TrajectoryFollowerCommand(
                drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(DISTANCE)
                        .build(), drive
        );
        schedule(new WaitUntilCommand(this::isStarted).andThen(straightFollower.whenFinished(() -> {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        })));
    }

}