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
import org.firstinspires.ftc.teamcode.Commands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeTakeIn;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotateToggle;
import org.firstinspires.ftc.teamcode.Commands.drone.DroneLauncherSetState;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.ArmGetToSelectedPosition;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.SetRobotSideCenter;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.SetRobotSideLeft;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.SetRobotSideRight;
import org.firstinspires.ftc.teamcode.Commands.turret.RotateTurretByPID;
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
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        initArm();
        initGamepad();

    }

    public void initGamepad() {
        gamepadEx1 = new GamepadEx(gamepad1);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new RotateTurretByPID(turret, 0));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new RotateTurretByPID(turret, -90));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new RotateTurretByPID(turret, 90));


//        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.INTAKE, false));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SAFE_PLACE, false));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.TEST_POSITION, false));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SECOND_TEST_POSITION, false));

        /** TODO: CHECK THIS BEFORE CHANGING
         *         gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SetRobotSide(elevator, elbow, extender, turret, antiTurret, Side.LEFT));
         *         gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new SetRobotSide(elevator, elbow, extender, turret, antiTurret, Side.RIGHT));
         *         gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SetRobotSide(elevator, elbow, extender, turret, antiTurret, Side.CENTER));
         */


    }

    public void initArm(){
        initTurret();
//        initElevator();
        initElbow();
//        initExtender();
//        initAntiTurret();
//        initCartridge();

//        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.INTAKE, false).schedule();
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