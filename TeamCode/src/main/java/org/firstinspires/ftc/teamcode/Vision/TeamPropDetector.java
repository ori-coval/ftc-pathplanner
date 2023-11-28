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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;

public class TeamPropDetector extends OpenCvPipeline {
    private final AllianceColor allianceColor;
    private Side teamPropSide;
    private final Rect rightRectangle = new Rect(426, 0, 213, 480);
    private final Rect leftRectangle = new Rect(0,  0,213,480);
    private Rect centerRectangle = new Rect(213, 0, 213, 480);

    private Scalar leftTotalRGBColors;
    private Scalar rightTotalRGBColors;
    private Scalar centerTotalRGBColors;
    private final double RGBValuesNormalizer = 8 / Math.pow(10,7);

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

        leftTotalRGBColors = new Scalar(0, 0, 0, 0);
        rightTotalRGBColors = new Scalar(0, 0, 0, 0);
        centerTotalRGBColors = new Scalar(0, 0, 0, 0);
    }




    @Override
    public void init(Mat mat) {
    }

    @Override
    public Mat processFrame(Mat frame) {
        Mat leftMat = frame.submat(leftRectangle);
        Mat rightMat = frame.submat(rightRectangle);
        Mat centerMat = frame.submat(centerRectangle);

        leftTotalRGBColors = Core.sumElems(leftMat);
        rightTotalRGBColors = Core.sumElems(rightMat);
        centerTotalRGBColors = Core.sumElems(centerMat);

        switch (allianceColor){
            case RED:
                if (getNormalizedColorFromScalar(leftTotalRGBColors, 0)>redTolerances.get(Side.LEFT)){
                    teamPropSide = Side.LEFT;
                } else if (getNormalizedColorFromScalar(rightTotalRGBColors, 0)>redTolerances.get(Side.RIGHT)){
                    teamPropSide = Side.RIGHT;
                } else if (getNormalizedColorFromScalar(centerTotalRGBColors, 0)>redTolerances.get(Side.CENTER)) {
                    teamPropSide = Side.CENTER;
                }
                break;
            case BLUE:
                if (getNormalizedColorFromScalar(leftTotalRGBColors, 2)>blueTolerances.get(Side.LEFT)){
                    teamPropSide = Side.LEFT;
                } else if (getNormalizedColorFromScalar(rightTotalRGBColors, 2)>blueTolerances.get(Side.RIGHT)){
                    teamPropSide = Side.RIGHT;
                } else if (getNormalizedColorFromScalar(centerTotalRGBColors, 2)>blueTolerances.get(Side.CENTER)) {
                    teamPropSide = Side.CENTER;
                }
                break;
        }
        return rightMat;
    }

    public Side getSide() {
        return teamPropSide;
    }

    private double getNormalizedColorFromScalar(Scalar RGBColors, int RGBIndex){
        if (RGBColors.val == null){
            return -1;
        }
        return RGBColors.val[RGBIndex] * RGBValuesNormalizer;
    }

    public double getSideColor(Side side, int RGBIndex){
        switch (side){
            case LEFT:
                return getNormalizedColorFromScalar(leftTotalRGBColors, RGBIndex);
            case RIGHT:
                return getNormalizedColorFromScalar(rightTotalRGBColors, RGBIndex);
            case CENTER:
                return getNormalizedColorFromScalar(centerTotalRGBColors, RGBIndex);
        }
        return -1;
    }
}
