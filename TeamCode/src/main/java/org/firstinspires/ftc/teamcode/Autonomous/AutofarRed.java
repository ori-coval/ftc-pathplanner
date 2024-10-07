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
        Pose2d beginPose = new Pose2d(-15.36, 62.68 , 90);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
        drive.actionBuilder(beginPose)
        .splineTo(new Vector2d(-41.57, -55.98), Math.toRadians(180.00))
                .waitSeconds(200)
        .splineTo(new Vector2d(-55.98, -55.4), Math.toRadians(90.00))
                .waitSeconds(700)
        .splineTo(new Vector2d(36.76 , -28.9) , Math.toRadians(16.19))
                .waitSeconds(700)
        .splineTo(new Vector2d(-56.13 , -61.08) , Math.toRadians(199.11))
                .waitSeconds(700)


                .build());

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

    }
}