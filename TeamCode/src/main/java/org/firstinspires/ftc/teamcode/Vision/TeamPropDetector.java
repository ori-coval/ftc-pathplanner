package org.firstinspires.ftc.teamcode.Vision;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.Configuration;
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
    OpenCvCamera webcam;
    private final AllianceColor allianceColor;
    private Side teamPropSide = null;

    private final Rect leftRectangle;
    private final Rect centerRectangle;

    private double leftMean;
    private double centerMean;

    private final Mat YCrCb = new Mat();
    private final Mat blurredImage = new Mat();
    private final Mat currentColorChannel = new Mat();

    public enum Tolerance {
        RED_CENTER(133), RED_LEFT(138), BLUE_CENTER(0), BLUE_LEFT(0);

        final double tolerance;

        Tolerance(double tolerance) {
            this.tolerance = tolerance;
        }
    }

    public TeamPropDetector(HardwareMap hardwareMap, AllianceColor allianceColor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.allianceColor = allianceColor;

        centerRectangle = new Rect(310,  279,270,200);
        leftRectangle = new Rect(0, 279, 160, 200);

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
        Mat leftMat = currentColorChannel.submat(leftRectangle);
        Mat centerMat = currentColorChannel.submat(centerRectangle);


        // get the area specific means
        leftMean = Core.mean(leftMat).val[0];
        centerMean = Core.mean(centerMat).val[0];

        double centerTolerance = (allianceColor == AllianceColor.BLUE) ? Tolerance.BLUE_CENTER.tolerance : Tolerance.RED_CENTER.tolerance;
        double leftTolerance = (allianceColor == AllianceColor.BLUE) ? Tolerance.BLUE_LEFT.tolerance : Tolerance.RED_LEFT.tolerance;

        // choose the most prominent side
        if((centerMean > centerTolerance) || (leftMean > leftTolerance)) {
            if(centerMean > centerTolerance) {
                teamPropSide = Side.CENTER;
            } else teamPropSide = Side.LEFT;
        } else {
            teamPropSide = Side.RIGHT;
        }

        // visual indicator
        Imgproc.rectangle(frame, leftRectangle, new Scalar(255, 0, 0), 5);
        Imgproc.rectangle(frame, centerRectangle, new Scalar(0, 0, 255), 5);

        //Camera Stream output
        return frame;
    }

    public Side getTeamPropSide() {
        return teamPropSide;
    }

    public void telemetry() {
        telemetry.addData("allianceColor", allianceColor);
        telemetry.addData("leftMean", leftMean);
        telemetry.addData("centerMean", centerMean);
        telemetry.addData("teamPropSide", teamPropSide);
        telemetry.update();
    }

}