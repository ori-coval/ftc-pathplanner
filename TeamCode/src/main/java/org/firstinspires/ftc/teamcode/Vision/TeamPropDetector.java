package org.firstinspires.ftc.teamcode.Vision;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.Configuration;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;
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
import org.openftc.easyopencv.OpenCvWebcam;

public class TeamPropDetector extends OpenCvPipeline {
    Telemetry telemetry;
    public OpenCvWebcam webcam;
    private final RobotControl robot;
    private DetectionSide teamPropSide = null;

    private final Rect farRectangle;
    private final Rect centerRectangle;

    private double farMean;
    private double centerMean;

    private double centerTolerance;
    private double farTolerance;

    private final Mat YCrCb = new Mat();
    private final Mat blurredImage = new Mat();
    private final Mat currentColorChannel = new Mat();

    public enum FarTolerance {
        RED_CENTER(131), RED_FAR(134), BLUE_CENTER(129), BLUE_FAR(129);

        double tolerance;

        FarTolerance(double tolerance) {
            this.tolerance = tolerance;
        }
    }

    public enum CloseTolerance {
        RED_CENTER(131), RED_FAR(133), BLUE_CENTER(128), BLUE_FAR(129);

        double tolerance;

        CloseTolerance(double tolerance) {
            this.tolerance = tolerance;
        }
    }

    public TeamPropDetector(GamepadEx gamepad1Ex, HardwareMap hardwareMap, RobotControl robot, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.robot = robot;

        //Gamepad
        if(robot.robotSide == AllianceSide.FAR) {
            if(robot.allianceColor == AllianceColor.RED) {
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> FarTolerance.RED_CENTER.tolerance++));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> FarTolerance.RED_CENTER.tolerance--));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> FarTolerance.RED_FAR.tolerance++));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> FarTolerance.RED_FAR.tolerance--));
            } else {
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> FarTolerance.BLUE_CENTER.tolerance++));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> FarTolerance.BLUE_CENTER.tolerance--));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> FarTolerance.BLUE_CENTER.tolerance++));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> FarTolerance.BLUE_CENTER.tolerance--));
            }
        } else {
            if(robot.allianceColor == AllianceColor.RED) {
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> CloseTolerance.RED_CENTER.tolerance++));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> CloseTolerance.RED_CENTER.tolerance--));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> CloseTolerance.RED_FAR.tolerance++));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> CloseTolerance.RED_FAR.tolerance--));
            } else {
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> CloseTolerance.BLUE_CENTER.tolerance++));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> CloseTolerance.BLUE_CENTER.tolerance--));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> CloseTolerance.BLUE_CENTER.tolerance++));
                gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> CloseTolerance.BLUE_CENTER.tolerance--));
            }
        }

        if(robot.robotSide == AllianceSide.FAR) {
            if(robot.allianceColor == AllianceColor.RED) {
                centerRectangle = new Rect(310,  279,270,200);
                farRectangle = new Rect(0, 279, 160, 200);
            } else {
                centerRectangle = new Rect(0,  279,330,200);
                farRectangle = new Rect(445, 279, 160, 200);
            }
        } else {
            if(robot.allianceColor == AllianceColor.RED) { //done
                centerRectangle = new Rect(40, 280, 300, 199);
                farRectangle = new Rect(400, 240, 239, 239);
            } else {
                centerRectangle = new Rect(300,  240,339,239);
                farRectangle = new Rect(0, 240, 200, 239);
            }
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, Configuration.WEB_CAM), cameraMonitorViewId);


        webcam.setPipeline(this);

        webcam.setMillisecondsPermissionTimeout(5000);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });


    }

    @Override
    public Mat processFrame(Mat frame) {
        // blur image
        Imgproc.GaussianBlur(frame, blurredImage, new Size(15,15), 0.0);

        // change image format in order to separate the blue and red channels
        Imgproc.cvtColor(blurredImage, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        // choose wanted channel
        switch (robot.allianceColor){
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

        if(robot.robotSide == AllianceSide.FAR) {
            centerTolerance = (robot.allianceColor == AllianceColor.BLUE) ? FarTolerance.BLUE_CENTER.tolerance : FarTolerance.RED_CENTER.tolerance;
            farTolerance = (robot.allianceColor == AllianceColor.BLUE) ? FarTolerance.BLUE_FAR.tolerance : FarTolerance.RED_FAR.tolerance;
        } else {
            centerTolerance = (robot.allianceColor == AllianceColor.BLUE) ? CloseTolerance.BLUE_CENTER.tolerance : FarTolerance.RED_CENTER.tolerance;
            farTolerance = (robot.allianceColor == AllianceColor.BLUE) ? CloseTolerance.BLUE_FAR.tolerance : FarTolerance.RED_FAR.tolerance;
        }

        double centerDistance = Math.abs(centerMean - centerTolerance);
        double farDistance = Math.abs(farMean - farTolerance);


        // choose the most prominent side
        DetectionSide tempResult = DetectionSide.CLOSE;

        if(centerMean > centerTolerance && farMean > farTolerance) {
            tempResult = (centerDistance > farDistance) ? DetectionSide.CENTER : DetectionSide.FAR;
        } else if(centerMean > centerTolerance) {
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
        telemetry.addData("farMean", farMean);
        telemetry.addData("farTolerance", farTolerance);
        telemetry.addData("centerMean", centerMean);
        telemetry.addData("centerTolerance", centerTolerance);
        telemetry.update();
    }

}