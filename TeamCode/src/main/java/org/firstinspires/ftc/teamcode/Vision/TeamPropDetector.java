package org.firstinspires.ftc.teamcode.Vision;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    OpenCvCamera webcam;
    private final AllianceColor allianceColor;
    public Side teamPropSide;

    private final Rect rightRectangle = new Rect(426, 0, 213, 480);
    private final Rect leftRectangle = new Rect(0,  0,213,480);
    private final Rect centerRectangle = new Rect(213, 0, 213, 480);

    private final Mat YCrCb = new Mat();
    private final Mat blurredImage = new Mat();
    private final Mat currentColorChannel = new Mat();

    double leftOffset = 0;
    double centerOffset = 0;
    double rightOffset = 0;

    public TeamPropDetector(HardwareMap hardwareMap, AllianceColor allianceColor) {
        this.allianceColor = allianceColor;

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
            case RED: Core.extractChannel(YCrCb, currentColorChannel, 1);
            case BLUE: Core.extractChannel(YCrCb, currentColorChannel, 2);
        }

        // get the area specific means and offset them accordingly
        double leftMean   = Core.mean(currentColorChannel.submat(leftRectangle  )).val[0] - leftOffset  ;
        double centerMean = Core.mean(currentColorChannel.submat(centerRectangle)).val[0] - centerOffset;
        double rightMean  = Core.mean(currentColorChannel.submat(rightRectangle )).val[0] - rightOffset ;

        // choose the most prominent side
        if (leftMean > centerMean && leftMean > rightMean) teamPropSide = Side.LEFT;
        else if (centerMean > rightMean)                   teamPropSide = Side.CENTER;
        else                                               teamPropSide = Side.RIGHT;


        // visual indicator
        Imgproc.rectangle(frame, leftRectangle, new Scalar(255, 0, 0), 5);
        Imgproc.rectangle(frame, centerRectangle, new Scalar(0, 255, 0), 5);
        Imgproc.rectangle(frame, rightRectangle, new Scalar(0, 0, 255), 5);

        //Camera Stream output
        return frame;
    }
}