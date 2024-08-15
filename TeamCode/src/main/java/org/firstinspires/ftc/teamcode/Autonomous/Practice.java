package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;

public class Practice extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,Math.toRadians(0)));


        Action trajectoryAction = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(10,10),Math.PI/2).build();












    }
}
