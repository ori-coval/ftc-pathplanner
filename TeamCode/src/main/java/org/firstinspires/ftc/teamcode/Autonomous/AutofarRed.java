package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;


@Config
@Autonomous(name = "AutofarRed", group = "Autonomous")

public final class AutofarRed extends CommandOpMode {

    int joegnf = 0;

    public  MecanumDrive drive;

    public  Pose2d beginPose;

    public Action traj;

    @Override
    public void initialize() {

        beginPose = new Pose2d(0, 0 , 0);
        drive = new MecanumDrive(hardwareMap, beginPose);


        traj = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(20, 10), Math.toRadians(0))
                //.waitSeconds(200)
//        .splineTo(new Vector2d(-55.98, -55.4), Math.toRadians(90.00))
//                //.waitSeconds(700)
//        .splineTo(new Vector2d(36.76 , -28.9) , Math.toRadians(16.19))
//                //.waitSeconds(700)
//        .splineTo(new Vector2d(-56.13 , -61.08) , Math.toRadians(199.11))
                //.waitSeconds(700)

                .build();


    }

    @Override
    public void run() {
        super.run();

        FtcDashboard.getInstance().getTelemetry().addData("x", drive.odo.getPosX());
        FtcDashboard.getInstance().getTelemetry().addData("y", drive.odo.getPosY());
        FtcDashboard.getInstance().getTelemetry().addData("heading" , drive.odo.getHeading());
        FtcDashboard.getInstance().getTelemetry().update();

        if(joegnf ==0){
            joegnf++;

            Actions.runBlocking(traj);




        }


    }
}