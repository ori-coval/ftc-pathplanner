package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.armCommands.antiTurret.AntiTurretGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.ScoringBothPixels;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.ScoringFirstPixel;
import org.firstinspires.ftc.teamcode.Commands.armCommands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToSelectedPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.BackToIntake;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.SetRobotSide;
import org.firstinspires.ftc.teamcode.Commands.auto.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.drivetrain.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.drone.DroneLauncherSetState;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeTakeIn;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotateToggle;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.Vision.AllianceColor;
import org.firstinspires.ftc.teamcode.Vision.TeamPropDetector;

import java.util.function.BooleanSupplier;

public class RobotControl extends Robot {
    OpModeType opModeType;

    HardwareMap hardwareMap;
    DriveTrain driveTrain;
    public SampleMecanumDrive autoDriveTrain;
    Elbow elbow;
    Turret turret;
    AntiTurret antiTurret;
    Cartridge cartridge;
    DroneLauncher droneLauncher;
    Elevator elevator;
    TeamPropDetector teamPropDetector;
    Gamepad gamepad1;
    Gamepad gamepad2;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;
    Extender extender;
    public Intake intake;
    private final double TRIGGER_THRESHOLD = 0.5;

    public enum OpModeType {
        TELEOP, AUTO
    }

    public RobotControl(OpModeType type, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        opModeType = type;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        reset(); //reset the scheduler

        initDriveTrain();
        initIntake();
        initArm();

        if(type == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    public void initTele() {
        initDroneLauncher();
        initGamepad();
    }

    public void initAuto() {
//        initVision();
        Pose2d startPose = new Pose2d(-63, 35, 0);
        autoDriveTrain.setPoseEstimate(startPose);
        Trajectories.init(autoDriveTrain, startPose);
    }


    //This is the one and only time I'll ever use regions
    //region SubSystemsInit
    public void initGamepad() {
        gamepadEx1 = new GamepadEx(gamepad1);

        BooleanSupplier rightTriggerCondition = () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD;
        BooleanSupplier leftTriggerCondition = () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD;

        Trigger rightTrigger1 = new Trigger(rightTriggerCondition);
        Trigger leftTrigger1 = new Trigger(leftTriggerCondition);

        leftTrigger1.whenActive(new ScoringFirstPixel(cartridge, leftTriggerCondition));
        rightTrigger1.whenActive(new ScoringBothPixels(elevator, elbow, extender, turret, antiTurret, cartridge, rightTriggerCondition));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SetRobotSide(elevator, elbow, extender, turret, antiTurret, cartridge, Side.LEFT));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new SetRobotSide(elevator, elbow, extender, turret, antiTurret, cartridge, Side.RIGHT));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SetRobotSide(elevator, elbow, extender, turret, antiTurret, cartridge, Side.CENTER));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ArmGetToSelectedPosition(elevator, elbow, extender, turret, antiTurret));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new IntakeRotateToggle(intake.roller));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new IntakeTakeIn(intake.lifter, intake.roller));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new BackToIntake(elevator, elbow, extender, turret, antiTurret, cartridge));


        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new DroneLauncherSetState(droneLauncher, DroneLauncher.State.RELEASE));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(ArmPositionSelector::moveUp));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(ArmPositionSelector::moveRight));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(ArmPositionSelector::moveDown));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(ArmPositionSelector::moveLeft));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.PRE_CLIMB, false));
    }

    public void initArm(){
        initTurret();
        initElevator();
        initElbow();
        initExtender();
        initAntiTurret();
        initCartridge();

        new ExtenderSetPosition(extender, ArmPosition.INTAKE.getExtenderPosition()).schedule();
        new AntiTurretGetToPosition(antiTurret, ArmPosition.INTAKE.getAntiTurretPosition()).schedule();
        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.INTAKE, false).schedule();
    }

    public void initDriveTrain() {
        if(opModeType == OpModeType.TELEOP) {
            driveTrain = new DriveTrain(hardwareMap);
            driveTrain.setDefaultCommand(new TeleopDriveCommand(driveTrain, gamepad1));
        } else {
            autoDriveTrain = new SampleMecanumDrive(hardwareMap);
        }
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
    public void initVision() { teamPropDetector = new TeamPropDetector(hardwareMap, AllianceColor.BLUE); }
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
        cartridge = new Cartridge(hardwareMap);
        cartridge.setState(Cartridge.State.OPEN);
    }
    public void initDroneLauncher() {
        droneLauncher = new DroneLauncher(hardwareMap);
    }
    //endregion

}
