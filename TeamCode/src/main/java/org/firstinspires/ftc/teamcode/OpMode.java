package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.drivetrain.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotateToggle;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.ArmGetToSelectedPosition;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.SetRobotSideCenter;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.SetRobotSideRightLeft;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeTakeIn;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.AllianceColor;
import org.firstinspires.ftc.teamcode.Vision.TeamPropDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "OpMode")
public class OpMode extends CommandOpMode {

    DriveTrain driveTrain;
    Elbow elbow;
    Turret turret;
    AntiTurret antiTurret;
    Cartridge cartridge;
    Elevator elevator;
    BNO055IMU imu;
    TeamPropDetector teamPropDetector;
    OpenCvCamera webcam;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;
    Extender extender;
    Intake intake;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

//        initDriveTrain();
//        initIntake();
        initElevator();
        initElbow();
        initTurret();
        initExtender();
        initAntiTurret();
        initGamepad();

        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.INTAKE, true).schedule();
    }

    public void initGamepad() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        initCartridge();

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new IntakeRotateToggle(intake.roller));

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ExtenderSetPosition(extender, Extender.Position.CLOSED));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ExtenderSetPosition(extender, Extender.Position.MID_WAY));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ExtenderSetPosition(extender, Extender.Position.OPEN));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SAFE_PLACE, true));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORE_TOP_CLOSE, true));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.INTAKE, true));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORING, true));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new IntakeTakeIn(intake.lifter, intake.roller));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new IntakeRotateToggle(intake.roller));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(ArmPositionSelector::moveUp));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(ArmPositionSelector::moveRight));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(ArmPositionSelector::moveDown));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(ArmPositionSelector::moveLeft));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SetRobotSideRightLeft(elevator, elbow, extender, turret, antiTurret, Side.LEFT));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new SetRobotSideRightLeft(elevator, elbow, extender, turret, antiTurret, Side.RIGHT));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SetRobotSideCenter(elevator, elbow, extender, turret, antiTurret));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> ArmPositionSelector.setRobotSide(Side.LEFT)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> ArmPositionSelector.setRobotSide(Side.RIGHT)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> ArmPositionSelector.setRobotSide(Side.CENTER)));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ArmGetToSelectedPosition(elevator, elbow, extender, turret, antiTurret));



    }


    public void initDriveTrain() {
        initIMU();
        driveTrain = new DriveTrain(hardwareMap, imu);
        driveTrain.setDefaultCommand(new TeleopDriveCommand(driveTrain, gamepad1));
    }

    public void initIntake() {
        intake = new Intake(hardwareMap);
    }

    public void initTurret() {
        turret = new Turret(hardwareMap);
    }

    public void initAntiTurret() {
        antiTurret = new AntiTurret(hardwareMap);
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

    public void initElevator() {
        elevator = new Elevator(hardwareMap);
    }

    public void initElbow() {
        elbow = new Elbow(hardwareMap);
    }

    public void initExtender() {
        extender = new Extender(hardwareMap);
    }

    public void initCartridge() {
        cartridge = new Cartridge(hardwareMap, gamepadEx1);
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
//        telemetry.addData("Pixel Count", intake.roller.getPixelCount());
//        telemetry.addData("Switch State", intake.roller.currentSwitchState());
//        telemetry.addData("isRobotFull", intake.roller.isRobotFull());


        ArmPositionSelector.telemetry(telemetry);

        telemetry.addData("selectedPosition", ArmPositionSelector.getPosition());
        telemetry.addData("isLeftOfBoard", ArmPositionSelector.getIsLeftOfBoard());
        telemetry.update();
    }
}
