package org.firstinspires.ftc.teamcode.Libraries.RoadRunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;




public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    public void runOpMode(Pose2d beginPose) throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Vector2d(0, 0));

            waitForStart();

            while (opModeIsActive()) {
                final Pose2d pose2d = new Pose2d(0, 0, 0);
                Actions.runBlocking(
                    drive.actionBuilder(pose2d, beginPose, beginPose)
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
