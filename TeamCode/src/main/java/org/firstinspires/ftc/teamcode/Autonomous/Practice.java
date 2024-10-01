package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;


@Config
@Autonomous(name = "practice", group = "Autonomous")

public final class Practice extends LinearOpMode {


   //public final GoBildaPinpointDriver;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 60, 90);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
        drive.actionBuilder(beginPose)
        .splineTo(new Vector2d(30, 30), Math.PI / 2)
        .splineTo(new Vector2d(0, -20), Math.PI)
                .build());
    }
}