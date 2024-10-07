package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;


@Config
@Autonomous(name = "AutocloseRed", group = "Autonomous")

public final class AutocloseRed extends LinearOpMode {


    //public final GoBildaPinpointDriver;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-15.04, -62.57, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(-18.65, -56.96), Math.toRadians(158.20))
                        .waitSeconds(200)
                        .splineTo(new Vector2d(-24.47, -55.75), Math.toRadians(180.00))
                        .waitSeconds(700)
                        .splineTo(new Vector2d(-29.08, -55.15), Math.toRadians(183.58))
                        .waitSeconds(700)
                        .splineTo(new Vector2d(-38.11, -55.75), Math.toRadians(197.14))
                        .waitSeconds(700)
                        .splineTo((new Vector2d(-45.53, -54.15)), Math.toRadians(177.14))
                        .waitSeconds(700)
                        .splineTo((new Vector2d(-45.53, -54.15)), Math.toRadians(177.14))
                        .waitSeconds(700)
                        .splineTo(new Vector2d(-49.14, -54.35), Math.toRadians(201.45))
                        .waitSeconds(700)
                        .splineTo(new Vector2d(-52.75, -56.76), Math.toRadians(225.00))
                        .waitSeconds(700)
                        .splineTo(new Vector2d(-58.96, -38.31), Math.toRadians(90.00))
                        .waitSeconds(700)
                        .splineTo(new Vector2d(-52.75, -56.96), Math.toRadians(225.00))

                        .build());

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

    }
}
