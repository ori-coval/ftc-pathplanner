package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;


@Config
@Autonomous(name = "AutofarRed", group = "Autonomous")

public final class AutofarRed extends LinearOpMode {


   //public final GoBildaPinpointDriver;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(57.15, -50.45, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
        drive.actionBuilder(beginPose)
        .splineTo(new Vector2d(56.57, 43.46), Math.PI / 2)
                .waitSeconds(200)
        .splineTo(new Vector2d(0, -20), Math.PI / 2)
                .build());

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

    }
}