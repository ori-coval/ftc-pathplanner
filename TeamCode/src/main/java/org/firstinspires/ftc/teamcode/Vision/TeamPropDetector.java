package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;

public class TeamPropDetector extends OpenCvPipeline {
    private final AllianceColor allianceColor;
    private Side teamPropSide;
    private final Rect rightRectangle = new Rect(426, 0, 213, 480);
    private final Rect leftRectangle = new Rect(0,  0,213,480);
    private Rect centerRectangle = new Rect(213, 0, 213, 480);

    private Mat YCrCb = new Mat();
    private Mat Cr = new Mat();
    private Mat Cb = new Mat();

    private Scalar blueLeft;
    private Scalar redLeft;
    private Scalar blueRight;
    private Scalar redRight;
    private Scalar blueCenter;
    private Scalar redCenter;

//    private final double RGBValuesNormalizer = 1 / Math.pow(10,6);

    private static HashMap<Side, Double> blueTolerances;
    private static HashMap<Side, Double> redTolerances;


    public TeamPropDetector(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
        blueTolerances = new HashMap<>();
        redTolerances = new HashMap<>();

        blueTolerances.put(Side.LEFT,0.0);
        blueTolerances.put(Side.RIGHT,0.0);
        blueTolerances.put(Side.CENTER,0.0);
        redTolerances.put(Side.LEFT, 0.0);
        redTolerances.put(Side.RIGHT, 0.0);
        redTolerances.put(Side.CENTER, 0.0);

        blueLeft = new Scalar(0, 0, 0, 0);
        redLeft = new Scalar(0, 0, 0, 0);
        blueRight = new Scalar(0, 0, 0, 0);
        redRight = new Scalar(0, 0, 0, 0);
        blueCenter = new Scalar(0, 0, 0, 0);
        redCenter = new Scalar(0, 0, 0, 0);
    }




    @Override
    public void init(Mat mat) {
    }

    @Override
    public Mat processFrame(Mat frame) {

        Imgproc.cvtColor(frame, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cr, 1);
        Core.extractChannel(YCrCb, Cb, 2);

        Mat leftMatCr = Cr.submat(leftRectangle);
        Mat leftMatCb = Cb.submat(leftRectangle);
        Mat rightMatCr = Cr.submat(rightRectangle);
        Mat rightMatCb = Cb.submat(rightRectangle);
        Mat centerMatCr = Cr.submat(centerRectangle);
        Mat centerMatCb = Cb.submat(centerRectangle);

        Imgproc.GaussianBlur(frame, frame, new Size(5,5), 0.0);
        Imgproc.rectangle(frame, leftRectangle, new Scalar(255, 0, 0), 5);
        Imgproc.rectangle(frame, centerRectangle, new Scalar(0, 255, 0), 5);
        Imgproc.rectangle(frame, rightRectangle, new Scalar(0, 0, 255), 5);

        redLeft = Core.mean(leftMatCr);
        blueLeft = Core.mean(leftMatCb);
        redRight = Core.mean(rightMatCr);
        blueRight = Core.mean(rightMatCb);
        redCenter = Core.mean(centerMatCr);
        blueCenter = Core.mean(centerMatCb);

        switch (allianceColor){
            case RED:
                if (getSideColor(Side.LEFT, 1)>redTolerances.get(Side.LEFT)){
                    teamPropSide = Side.LEFT;
                } else if (getSideColor(Side.RIGHT, 1)>redTolerances.get(Side.RIGHT)){
                    teamPropSide = Side.RIGHT;
                } else if (getSideColor(Side.CENTER, 1)>redTolerances.get(Side.CENTER)) {
                    teamPropSide = Side.CENTER;
                }
                break;
            case BLUE:
                if (getSideColor(Side.LEFT, 2)>blueTolerances.get(Side.LEFT)){
                    teamPropSide = Side.LEFT;
                } else if (getSideColor(Side.RIGHT, 2)>blueTolerances.get(Side.RIGHT)){
                    teamPropSide = Side.RIGHT;
                } else if (getSideColor(Side.CENTER, 2)>blueTolerances.get(Side.CENTER)) {
                    teamPropSide = Side.CENTER;
                }
                break;
        }
        return frame; //Camera Stream output
    }

    public Side getSide() {
        return teamPropSide;
    }

    /*
    private double getNormalizedColorFromScalar(Scalar RGBColors, int YCrCbIndex){
        if (RGBColors.val == null){
            return -1;
        }
        return RGBColors.val[YCrCbIndex] * RGBValuesNormalizer;
    }
    */

    public double getSideColor(Side side, int YCrCbIndex){
        switch (side){
            case LEFT:
                switch(YCrCbIndex) {
                    case 1:
                        return redLeft.val[0];
                    case 2:
                        return blueLeft.val[0];
                }
                return -1;
            case RIGHT:
                switch(YCrCbIndex) {
                    case 1:
                        return redRight.val[0];
                    case 2:
                        return blueRight.val[0];
                }
                return -1;
            case CENTER:
                switch(YCrCbIndex) {
                    case 1:
                        return redCenter.val[0];
                    case 2:
                        return blueCenter.val[0];
                }
                return -1;
        }
        return -1;
    }
}
