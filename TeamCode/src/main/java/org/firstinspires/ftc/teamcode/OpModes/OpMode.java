package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.Commands.drivetrain.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeTakeIn;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotateToggle;
import org.firstinspires.ftc.teamcode.Commands.drone.DroneLauncherSetState;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.ArmGetToSelectedPosition;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.SetRobotSideCenter;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.SetRobotSideLeft;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.SetRobotSideRight;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.AllianceColor;
import org.firstinspires.ftc.teamcode.Vision.TeamPropDetector;


@TeleOp(name = "OpMode")
public class OpMode extends CommandOpMode {

    DriveTrain driveTrain;
    Elbow elbow;
    Turret turret;
    AntiTurret antiTurret;
    Cartridge cartridge;
    DroneLauncher droneLauncher;
    Elevator elevator;
    TeamPropDetector teamPropDetector;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;
    Extender extender;
    Intake intake;

    private final double TRIGGER_THRESHOLD = 0.5;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        initDriveTrain();
//        initIntake();
//        initDroneLauncher();
//        initArm();
//        initGamepad();

    }

    public void initGamepad() {
        gamepadEx1 = new GamepadEx(gamepad1);

        boolean rightTriggerCondition = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD;
        boolean leftTriggerCondition = gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD;
        Trigger rightTrigger1 = new Trigger(() -> rightTriggerCondition);
        Trigger leftTrigger1 = new Trigger(() -> leftTriggerCondition);

        rightTrigger1.whileActiveOnce(getCartridgeCommand(Cartridge.State.OPEN, rightTriggerCondition));
        leftTrigger1.whileActiveOnce(getCartridgeCommand(Cartridge.State.SEMI_OPEN, leftTriggerCondition));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SetRobotSideRight(elevator, elbow, extender, turret, antiTurret));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new SetRobotSideLeft(elevator, elbow, extender, turret, antiTurret));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SetRobotSideCenter(elevator, elbow, extender, turret, antiTurret));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ArmGetToSelectedPosition(elevator, elbow, extender, turret, antiTurret));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new IntakeRotateToggle(intake.roller));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new IntakeTakeIn(intake.lifter, intake.roller));

        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new DroneLauncherSetState(droneLauncher, DroneLauncher.State.RELEASE));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.INTAKE, false));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(ArmPositionSelector::moveUp));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(ArmPositionSelector::moveRight));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(ArmPositionSelector::moveDown));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(ArmPositionSelector::moveLeft));
    }

    private Command getCartridgeCommand(Cartridge.State newState, boolean triggerCondition) {
        return new StartEndCommand(
                () -> cartridge.setState(newState),
                () -> {
                    cartridge.setState(Cartridge.State.CLOSED);
                    if(newState == Cartridge.State.OPEN) {
                        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORING, ArmPositionSelector.getIsLeftOfBoard()).schedule();
                    }
                },
                cartridge
        );
    }

    public void initArm(){
        initTurret();
        initElevator();
        initElbow();
        initExtender();
        initAntiTurret();
        initCartridge();

        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.INTAKE, false).schedule();
    }

    public void initDriveTrain() {
        driveTrain = new DriveTrain(hardwareMap);
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
    public void initVision() {
        teamPropDetector = new TeamPropDetector(hardwareMap, AllianceColor.BLUE);
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

    @Override
    public void run() {
        super.run();
        ArmPositionSelector.telemetry(telemetry);

        telemetry.addData("selectedPosition", ArmPositionSelector.getPosition());
        telemetry.addData("isLeftOfBoard", ArmPositionSelector.getIsLeftOfBoard());
        telemetry.update();
    }
}