package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.antiTurret.AntiTurretParallel;
import org.firstinspires.ftc.teamcode.Commands.drivetrain.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
import org.firstinspires.ftc.teamcode.SubSystems.Odometry;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.AllianceColor;
import org.firstinspires.ftc.teamcode.Vision.Side;
import org.firstinspires.ftc.teamcode.Vision.TeamPropDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "DriveTrain")
public class OpMode extends CommandOpMode {
    DriveTrain driveTrain;
    InTake inTake;
    Elbow elbow;
    Turret turret;
    AntiTurret antiTurret;
    BNO055IMU imu;

    TeamPropDetector teamPropDetector;
    OpenCvCamera webcam;
    Odometry odometry;
    GamepadEx gamepadEx1;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        IMUInit();
        DriveTrainInit();
        OdometryInit();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> odometry.resetLocation()));
    }

    public void DriveTrainInit() {
        IMUInit();
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("backLeftLin")
                , hardwareMap.dcMotor.get("backRightLin")
                , hardwareMap.dcMotor.get("frontRightLin")
                , hardwareMap.dcMotor.get("frontLeftLin")
                , imu);
        driveTrain.setDefaultCommand(new TeleopDriveCommand(driveTrain, gamepad1));
    }
    public void IMUInit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }
    public void IntakeInit() {

        inTake = new InTake(hardwareMap.dcMotor.get("inTake"));
    }
    public void ElbowInit() {
        elbow = new Elbow(hardwareMap.dcMotor.get("elbow"));

    }
    public void TurretInit() {
        turret = new Turret(
                hardwareMap.crservo.get("turretMotorA"),
                hardwareMap.crservo.get("turretMotorB"),
                hardwareMap.dcMotor.get("frontLeftLin")
        );
    }
    public void OdometryInit() {
        odometry = new Odometry(
                hardwareMap.dcMotor.get("frontLeftLin"),
                hardwareMap.dcMotor.get("backLeftLin")
        );
    }
    public void AntiTurretInit() {
        antiTurret = new AntiTurret(hardwareMap.servo.get("antiTurret"));
        antiTurret.setDefaultCommand(new AntiTurretParallel(antiTurret, () -> turret.getEncoderValue()));
    }
    public void VisionInit(){
        teamPropDetector = new TeamPropDetector(AllianceColor.BLUE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Weiss cam"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        webcam.setPipeline(teamPropDetector);
    }

    @Override
    public void run() {
        super.run();

        if (opModeIsActive()) {
            telemetry.addData("LeftBlue", teamPropDetector.getSideColor(Side.LEFT, 2));
            telemetry.addData("RightBlue", teamPropDetector.getSideColor(Side.RIGHT, 2));
            telemetry.addData("CenterBlue", teamPropDetector.getSideColor(Side.CENTER, 2));
            telemetry.addData("LeftRed", teamPropDetector.getSideColor(Side.LEFT, 1));
            telemetry.addData("RightRed", teamPropDetector.getSideColor(Side.RIGHT, 1));
            telemetry.addData("CenterRed", teamPropDetector.getSideColor(Side.CENTER, 1));
            telemetry.addData("Side", teamPropDetector.getSide());
            telemetry.update();
        }

//        telemetry.addData("odometry", odometry.getLocation());
        telemetry.update();
    }
}
