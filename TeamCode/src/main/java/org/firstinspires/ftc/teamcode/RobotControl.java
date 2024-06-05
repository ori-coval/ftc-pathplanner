package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.driveTrain.DriveCommand;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.AutoDriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class RobotControl extends Robot {
    OpModeType opModeType;
    public AllianceColor allianceColor;
    public Pose2d startPose;
    HardwareMap hardwareMap;

    //subsystems
    public DriveTrain driveTrain;
    public AutoDriveTrain autoDriveTrain;

    //gamepads
    Gamepad gamepad1;
    Gamepad gamepad2;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    public Telemetry telemetry;

    public static double lastHeading = 0;

    public enum OpModeType {
        TELEOP, AUTO, DEBUG
    }

    public RobotControl(OpModeType type, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        opModeType = type;
        initializeAttributes(type, hardwareMap, gamepad1, gamepad2, telemetry);
        initializeSystems(type);
    }

    public RobotControl(OpModeType type, AllianceColor allianceColor, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this(type, hardwareMap, gamepad1, gamepad2, telemetry);
        this.allianceColor = allianceColor;
    }

    private void initializeAttributes(OpModeType type, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        opModeType = type;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        reset(); //reset the scheduler
    }

    private void initializeSystems(OpModeType type) {
        if (type == OpModeType.TELEOP) {
            initTele();
        } else if (type == OpModeType.AUTO) {
            initAuto();
        } else {
            initDebug();
        }
    }

    public void initTele() {
        initDriveTrain();
        //init subsystems


        initGamepad();
    }

    public void initAuto() {
        initDriveTrain();
        initTrajectories();
    }

    private void initTrajectories() {
        startPose = new Pose2d();
        //set start position

        autoDriveTrain.setPoseEstimate(startPose);
    }

    public void initDebug() {
        //init subsystems
        initDebugGamepad();
    }


    public void initGamepad() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        //setup buttons

        //creating new triggers
        //Ex: BooleanSupplier rightTriggerCondition = () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD;


    }

    public void initDebugGamepad() {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        //setup debug buttons

    }

    //subsystem init methods

    public void initDriveTrain() {
        if (opModeType == OpModeType.TELEOP) {
            driveTrain = new DriveTrain(hardwareMap, lastHeading);
            register(driveTrain);
            driveTrain.setDefaultCommand(new DriveCommand(driveTrain, gamepad1));
        } else if (opModeType == OpModeType.AUTO) {
            autoDriveTrain = new AutoDriveTrain(new SampleMecanumDrive(hardwareMap));
        }
    }


}
