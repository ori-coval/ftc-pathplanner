package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "AutocloseRed", group = "Autonomous")
public class AutocloseRed extends CommandOpMode {

    Action traj;

    @Override
    public void initialize() {


        Pose2d initialPose = new Pose2d(0, 0, 90);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

         traj = drive.actionBuilder(initialPose)

                .turn(Math.toRadians(90))
                .lineToX(-50.93)
                .setTangent(Math.toRadians(180))

                .build();

    }

    @Override
    public void run() {


        super.run();

        Actions.runBlocking(

                traj
                
        );


    }
}

