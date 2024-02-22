package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.armCommands.antiTurret.AntiTurretGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.ScoringBothPixels;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.ScoringFirstPixel;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.Climb;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.ElevatorGoUp;
import org.firstinspires.ftc.teamcode.Commands.armCommands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToSelectedPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.BackToIntake;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.SetRobotSide;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.UnsafeMoveArm;
import org.firstinspires.ftc.teamcode.Commands.auto.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.drivetrain.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.drone.DroneLauncherSetState;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeTakeIn;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeEjectToggle;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotateToggle;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.ServoTuningCommand;
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
import org.firstinspires.ftc.teamcode.Utils.Configuration;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Vision.TeamPropDetector;

import java.util.function.BooleanSupplier;

public class RobotControl extends Robot {
    OpModeType opModeType;
    public AllianceColor allianceColor;
    public Side robotSide;
    public Pose2d startPose;
    HardwareMap hardwareMap;
    public DriveTrain driveTrain;
    public Elbow elbow;
    public Turret turret;
    public AntiTurret antiTurret;
    public Cartridge cartridge;
    public DroneLauncher droneLauncher;
    public Elevator elevator;
    public TeamPropDetector teamPropDetector;
    Gamepad gamepad1;
    Gamepad gamepad2;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;
    public Extender extender;
    public Intake intake;
    Telemetry telemetry;
    private final double TRIGGER_THRESHOLD = 0.5;

    public enum OpModeType {
        TELEOP, AUTO, DEBUG
    }

    public RobotControl(OpModeType type, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        opModeType = type;
        initializeAttributes(type, hardwareMap, gamepad1, gamepad2, telemetry);
        initializeSystems(type);
    }

    public RobotControl(OpModeType type, AllianceColor allianceColor, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        opModeType = type;
        this.allianceColor = allianceColor;
        initializeAttributes(type, hardwareMap, gamepad1, gamepad2, telemetry);
        initializeSystems(type);
    }

    public RobotControl(OpModeType type, AllianceColor allianceColor, Side robotSide, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        opModeType = type;
        this.allianceColor = allianceColor;
        this.robotSide = robotSide;
        initializeAttributes(type, hardwareMap, gamepad1, gamepad2, telemetry);
        initializeSystems(type);
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
        if(type == OpModeType.TELEOP) {
            initTele();
        } else if (type == OpModeType.AUTO) {
            initAuto();
        } else {
            initDebug();
        }
    }



    public void initTele() {
        initDriveTrain();
        initArm();
        initIntake();
        initDroneLauncher();
        initGamepad();

        cartridge.setState(Cartridge.State.OPEN);
    }

    public void initAuto() {
        initDriveTrain();
        initArm();
        initIntake();
//        intake.roller.setPixelCount(1);
        initVision();
        initTrajectories();
    }

    private void initTrajectories() {
        startPose = new Pose2d();
        if(allianceColor == AllianceColor.RED) {
            if(robotSide == Side.LEFT) {
                startPose = new Pose2d(-63, 38, 0);
            } else if(robotSide == Side.RIGHT) {
                startPose = new Pose2d(-63, -38 + 24, 0);
            }
        } else {
            if(robotSide == Side.LEFT) {
                startPose = new Pose2d(63, -14, Math.toRadians(180));
            } else if(robotSide == Side.RIGHT) {
                startPose = new Pose2d(63, 38, Math.toRadians(180));
            }
        }
        driveTrain.setPoseEstimate(startPose);
        Trajectories.init(this, startPose);
    }

    public void initDebug() {
        initTurret();
        initElevator();
        initElbow();
        initExtender();
        initAntiTurret();
        initCartridge();
        initExtender();
        initDebugGamepad();
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
        rightTrigger1.whenActive(new ScoringBothPixels(this, rightTriggerCondition));

/*        if(allianceColor == AllianceColor.RED) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SetRobotSide(this, Side.LEFT));
        } else {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SetRobotSide(this, Side.RIGHT));
        }*/

        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SetRobotSide(this, Side.CENTER));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ArmGetToSelectedPosition(this));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new IntakeRotateToggle(intake.roller));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new IntakeTakeIn(intake.lifter, intake.roller));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new BackToIntake(this));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new IntakeEjectToggle(intake.roller));


        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(ArmPositionSelector::moveUp));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(ArmPositionSelector::moveRight));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(ArmPositionSelector::moveDown));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(ArmPositionSelector::moveLeft));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whileActiveOnce(new Climb(this));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.X).whileActiveOnce(new ElevatorGoUp(this));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new DroneLauncherSetState(droneLauncher, DroneLauncher.State.RELEASE));



    }

    public void initDebugGamepad() {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);


        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whileActiveOnce(new ServoTuningCommand(hardwareMap, telemetry, gamepadEx1, Configuration.ANTI_TURRET));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whileActiveOnce(new ServoTuningCommand(hardwareMap, telemetry, gamepadEx1, Configuration.DRONE));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whileActiveOnce(new ServoTuningCommand(hardwareMap, telemetry, gamepadEx1, Configuration.CARTRIDGE));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileActiveOnce(new ServoTuningCommand(hardwareMap, telemetry, gamepadEx1, Configuration.ELBOW_RIGHT, Configuration.ELBOW_LEFT));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveOnce(new ServoTuningCommand(hardwareMap, telemetry, gamepadEx1, Configuration.DRONE));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileActiveOnce(new ServoTuningCommand(hardwareMap, telemetry, gamepadEx1, Configuration.INTAKE_SERVO));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileActiveOnce(new ServoTuningCommand(hardwareMap, telemetry, gamepadEx1, Configuration.EXTENDER));

        //        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new ExtenderSetPosition(this.extender, Extender.Position.OPEN));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ExtenderSetPosition(this.extender, Extender.Position.MID_WAY));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ExtenderSetPosition(this.extender, Extender.Position.CLOSED));
        ServoTuningCommand.telemetry(telemetry);

    }

    public void initArm() {
        initTurret();
        initElevator();
        initElbow();
        initExtender();
        initAntiTurret();
        initCartridge();

        new ExtenderSetPosition(extender, Extender.Position.CLOSED).schedule();
        new AntiTurretGetToPosition(antiTurret, ArmPosition.INTAKE.getAntiTurretPosition()).schedule();
        new UnsafeMoveArm(this, ArmPosition.INTAKE, false).schedule();
    }

    public void initDriveTrain() {
        driveTrain = new DriveTrain(new SampleMecanumDrive(hardwareMap), true);
        if(opModeType == OpModeType.TELEOP) {
            register(driveTrain);
            driveTrain.setDefaultCommand(new DriveCommand(
                    driveTrain, () -> (-gamepadEx1.getLeftY()), gamepadEx1::getLeftX, gamepadEx1::getRightX
            ));

            schedule(new RunCommand(() ->
                    driveTrain.update()
            ));

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
    public void initVision() {
        teamPropDetector = new TeamPropDetector(hardwareMap, AllianceColor.RED, telemetry);
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
        cartridge = new Cartridge(hardwareMap);
    }
    public void initDroneLauncher() {
        droneLauncher = new DroneLauncher(hardwareMap);
    }
    //endregion

}
