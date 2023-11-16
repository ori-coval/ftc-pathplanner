package org.firstinspires.ftc.teamcode.Vision;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "PipelineTest")
public class KooKyTest extends LinearOpMode {

    private TeamPropDetector teamPropDetector;
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
        teamPropDetector = new TeamPropDetector(AllianceColor.BLUE, telemetry);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1280, 720))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(teamPropDetector)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();


        while (opModeInInit()) {
            telemetry.addData("LeftBlue",teamPropDetector.leftTotalRGBColors.val[2]);
            telemetry.addData("RightBlue",teamPropDetector.rightTotalRGBColors.val[2]);
            telemetry.addData("CenterBlue",teamPropDetector.centerTotalRGBColors.val[2]);
            telemetry.addData("LeftRed",teamPropDetector.leftTotalRGBColors.val[0]);
            telemetry.addData("RightRed",teamPropDetector.rightTotalRGBColors.val[0]);

            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Location", teamPropDetector.getSide());
            telemetry.update();
        }
    }
}
