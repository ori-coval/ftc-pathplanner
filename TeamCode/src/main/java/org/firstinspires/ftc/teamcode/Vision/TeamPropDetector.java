package org.firstinspires.ftc.teamcode.Vision;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.Configuration;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDetector extends OpenCvPipeline {
    Telemetry telemetry;
    public OpenCvCamera webcam;
    private final AllianceColor allianceColor;
    private DetectionSide teamPropSide = null;

    private final Rect farRectangle;
    private final Rect centerRectangle;

    private double farMean;
    private double centerMean;

    private final Mat YCrCb = new Mat();
    private final Mat blurredImage = new Mat();
    private final Mat currentColorChannel = new Mat();

    public enum Tolerance {
        RED_CENTER(131), RED_FAR(134), BLUE_CENTER(125), BLUE_FAR(128);

        final double tolerance;

        Tolerance(double tolerance) {
            this.tolerance = tolerance;
        }
    }

    public TeamPropDetector(HardwareMap hardwareMap, AllianceColor allianceColor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;

        if(allianceColor == AllianceColor.RED) {
            centerRectangle = new Rect(310,  279,270,200);
            farRectangle = new Rect(0, 279, 160, 200); //LEFT (RED)
        } else {
            centerRectangle = new Rect(0,  279,300,200);
            farRectangle = new Rect(445, 279, 160, 200); //RIGHT (BLUE)
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, Configuration.WEB_CAM), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        webcam.setPipeline(this);

    }

    @Override
    public Mat processFrame(Mat frame) {
        // blur image
        Imgproc.GaussianBlur(frame, blurredImage, new Size(15,15), 0.0);

        // change image format in order to separate the blue and red channels
        Imgproc.cvtColor(blurredImage, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        // choose wanted channel
        switch (allianceColor){
            case RED: {
                Core.extractChannel(YCrCb, currentColorChannel, 1);
                break;
            }
            case BLUE: Core.extractChannel(YCrCb, currentColorChannel, 2);
        }

        // Submats from the current channel
        Mat farMat = currentColorChannel.submat(farRectangle);
        Mat centerMat = currentColorChannel.submat(centerRectangle);


        // get the area specific means
        farMean = Core.mean(farMat).val[0];
        centerMean = Core.mean(centerMat).val[0];

        double centerTolerance = (allianceColor == AllianceColor.BLUE) ? Tolerance.BLUE_CENTER.tolerance : Tolerance.RED_CENTER.tolerance;
        double farTolerance = (allianceColor == AllianceColor.BLUE) ? Tolerance.BLUE_FAR.tolerance : Tolerance.RED_FAR.tolerance;

        // choose the most prominent side
        DetectionSide tempResult = DetectionSide.CLOSE;

        if(centerMean > centerTolerance) {
            tempResult = DetectionSide.CENTER;
        } else if(farMean > farTolerance) {
            tempResult = DetectionSide.FAR;
        }

        teamPropSide = tempResult;

        // visual indicator
        Imgproc.rectangle(frame, farRectangle, new Scalar(255, 0, 0), 5);
        Imgproc.rectangle(frame, centerRectangle, new Scalar(0, 0, 255), 5);

        //Camera Stream output
        return frame;
    }

    public DetectionSide getTeamPropSide() {
        return teamPropSide;
    }

    public void telemetry() {
        telemetry.addData("teamPropSide", teamPropSide);
        telemetry.addData("allianceColor", allianceColor);
        telemetry.addData("farMean", farMean);
        telemetry.addData("centerMean", centerMean);
        telemetry.update();
    }

}