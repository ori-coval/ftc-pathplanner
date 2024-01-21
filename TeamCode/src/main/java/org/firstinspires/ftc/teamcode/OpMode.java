package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.antiTurret.AntiTurretParallel;
import org.firstinspires.ftc.teamcode.Commands.drivetrain.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeRotate;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
import org.firstinspires.ftc.teamcode.SubSystems.Odometry;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.AllianceColor;
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
    Cartridge cartridge;
    Conveyor conveyor;
    Elevator elevator;
    BNO055IMU imu;
    TeamPropDetector teamPropDetector;
    OpenCvCamera webcam;
    GamepadEx gamepadEx1;
    Extender extender;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        initIMU();
        initDriveTrain();
        initIntake();
        initElevator();
        initElbow();
        initConveyor();
        initExtender();
        initCartridge();


        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new IntakeRotate(inTake, -inTake.COLLECT_POWER));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new IntakeRotate(inTake, 0));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> inTake.setStackPosition(0)));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> inTake.setStackPosition(4)));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> inTake.setStackPosition(3)));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> inTake.setStackPosition(2)));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> inTake.setStackPosition(1)));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> cartridge.setState(Cartridge.State.CLOSED)));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> cartridge.setState(Cartridge.State.OPEN)));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> cartridge.setState(Cartridge.State.SEMI_OPEN)));


    }

    public void initDriveTrain() {
        initIMU();
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("backLeft")
                , hardwareMap.dcMotor.get("backRight")
                , hardwareMap.dcMotor.get("frontRight")
                , hardwareMap.dcMotor.get("frontLeft")
                , imu);
        driveTrain.setDefaultCommand(new TeleopDriveCommand(driveTrain, gamepad1));
    }
    public void initIntake() {
        inTake = new InTake((DcMotorEx) hardwareMap.dcMotor.get("inTake"), hardwareMap.servo.get("intakeServo"), gamepad1);
    }
    public void initTurret() {
        turret = new Turret(
                hardwareMap.crservo.get("turretRight"),
                hardwareMap.crservo.get("turretLeft"),
                hardwareMap.analogInput.get("turretEncoder")
        );
    }
    public void AntiTurretInit() {
        antiTurret = new AntiTurret(hardwareMap.servo.get("antiTurret"));
        antiTurret.setDefaultCommand(new AntiTurretParallel(antiTurret, () -> turret.getEncoderValue()));
    }
    public void VisionInit() {
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
    public void initConveyor() {
        conveyor = new Conveyor(0,
                hardwareMap.digitalChannel.get("switch")
        );

    }
    public void initElevator() {
        elevator = new Elevator(
                hardwareMap.dcMotor.get("elevatorDown"),
                hardwareMap.dcMotor.get("elevatorMid"),
                hardwareMap.dcMotor.get("elevatorUp"));
    }
    public void initElbow() {
        elbow = new Elbow(hardwareMap.servo.get("elbowRight"),hardwareMap.servo.get("elbowLeft"));

    }
    public void initExtender() {
        extender = new Extender(hardwareMap.servo.get("extender"));

    }
    public void initCartridge() {
        cartridge = new Cartridge(hardwareMap.servo.get("cartridge"));

    }
    public void initIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }


    @Override
    public void run() {
        super.run();

        telemetry.addData("Pixel Count", conveyor.getPixelCount());

    }
}
