package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;


@Config
@Autonomous(name = "AutocloseBlue", group = "Autonomous")

public final class AutocloseBlue extends LinearOpMode {


    //public final GoBildaPinpointDriver;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(23.67, 62.77, Math.toRadians(270.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(48.13, 54.35 ), Math.toRadians(25.97))
                        .waitSeconds(200)
                        .splineTo(new Vector2d(57.56, 55.55), Math.toRadians(45.00))
                        .waitSeconds(700)
                        .splineToConstantHeading(new Vector2d(58.56, 40.91), Math.toRadians(90.00))
                        .waitSeconds(700)
                        .splineTo(new Vector2d(60.37, 56.56), Math.toRadians(45.00))

                  .build());

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();
    }
}
