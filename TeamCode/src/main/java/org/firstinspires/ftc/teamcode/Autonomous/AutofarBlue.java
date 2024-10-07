package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;


@Config
@Autonomous(name = "AutofarBlue", group = "Autonomous")

public final class AutofarBlue extends LinearOpMode {


    //public final GoBildaPinpointDriver;

    @Override
    public void runOpMode() throws InterruptedException {
        Vector2d beginPose = new Vector2d(-15.36, 62.68);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
//                 .splineTo(new Vector2d(-54.24, 58.17), Math.toRadians(90))
//                     .waitSeconds(200)
                 .splineTo(new Vector2d(54.24, 58.17), Math.toRadians(90.00))
                     .waitSeconds(700)
                 .splineTo(new Vector2d(37.2 , 27.3) , Math.toRadians(90.00))
                     .waitSeconds(700)
                 .splineTo(new Vector2d(54.24, 58.17), Math.toRadians(90.00))
                     .waitSeconds(700)

                        .build());

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

    }
}