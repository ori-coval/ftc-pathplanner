package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.utilCommands.ServoTuningCommand;
import org.firstinspires.ftc.teamcode.Utils.Servos;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;


@TeleOp(name = "TuningOpMode")
public class TuningOpMode extends CommandOpMode {
    Elbow elbow;
    AntiTurret antiTurret;
    Cartridge cartridge;
    DroneLauncher droneLauncher;
    GamepadEx gamepadEx1;
    Extender extender;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        initElbow();
        initExtender();
        initAntiTurret();
        initDroneLauncher();
        initCartridge();
        initGamepad();

    }

    public void initGamepad() {
        gamepadEx1 = new GamepadEx(gamepad1);

        ServoTuningCommand servoTuning = new ServoTuningCommand(hardwareMap, telemetry, gamepadEx1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(servoTuning);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> servoTuning.setServo(Servos.ANTI_TURRET));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> servoTuning.setServo(Servos.CARTRIDGE));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> servoTuning.setServo(Servos.ELBOW));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> servoTuning.setServo(Servos.DRONE));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> servoTuning.setServo(Servos.INTAKE_LIFTER));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> servoTuning.setServo(Servos.EXTENDER));

        servoTuning.telemetry();
        telemetry.update();

    }
    public void initAntiTurret() {
        antiTurret = new AntiTurret(hardwareMap);
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
    }
}