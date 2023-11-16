package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Vision.AllianceColor;
import org.firstinspires.ftc.teamcode.Vision.Side;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.util.HashMap;

public class TeamPropDetector implements VisionProcessor {

    private final AllianceColor allianceColor;
    private Side teamPropSide;

    private final Rect rightRectangle = new Rect(852, 0, 426, 720);
    private final Rect leftRectangle = new Rect(0,0,426,720);
    private final Rect centerRectangle = new Rect(426,0,426,720);

    public Scalar leftTotalRGBColors;
    public Scalar rightTotalRGBColors;
    public Scalar centerTotalRGBColors;

    private static HashMap<Side, Double> blueTolerances= new HashMap<>();
    private static HashMap<Side, Double> redTolerances= new HashMap<>();

    Telemetry telemetry;

    static {
        blueTolerances.put(Side.LEFT,0.0);
        blueTolerances.put(Side.RIGHT,0.0);
        blueTolerances.put(Side.CENTER,0.0);
        redTolerances.put(Side.LEFT, 0.0);
        redTolerances.put(Side.RIGHT, 0.0);
        redTolerances.put(Side.CENTER, 0.0);
    }



    public TeamPropDetector(AllianceColor allianceColor, Telemetry telemetry) {
        this.allianceColor = allianceColor;
        this.telemetry = telemetry;

    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        telemetry.addData("init", "yes");
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        telemetry.addData("process", "yes");
        Mat leftMat = frame.submat(leftRectangle);
        Mat rightMat = frame.submat(rightRectangle);
        Mat centerMat = frame.submat(centerRectangle);

        leftTotalRGBColors = Core.sumElems(leftMat);
        rightTotalRGBColors = Core.sumElems(rightMat);
        centerTotalRGBColors = Core.sumElems(centerMat);

        switch (allianceColor){
            case RED:
                if (leftTotalRGBColors.val[0]>redTolerances.get(Side.LEFT)){
                    teamPropSide = Side.LEFT;
                } else if (rightTotalRGBColors.val[0]>redTolerances.get(Side.RIGHT)){
                    teamPropSide = Side.RIGHT;
                } else if (centerTotalRGBColors.val[0]>redTolerances.get(Side.CENTER))
                break;
            case BLUE:
                if (leftTotalRGBColors.val[2]>blueTolerances.get(Side.LEFT)){
                    teamPropSide = Side.LEFT;
                } else if (rightTotalRGBColors.val[2]>blueTolerances.get(Side.RIGHT)){
                    teamPropSide = Side.RIGHT;
                } else if (centerTotalRGBColors.val[2]>blueTolerances.get(Side.CENTER))
                break;
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Side getSide() {
        return teamPropSide;
    }


}
