package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.utils.ServoTuningCommand;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;

@TeleOp(name = "Servo Tuning")
public class OpMode extends CommandOpMode {

    Elbow elbow;
    AntiTurret antiTurret;
    Cartridge cartridge;
    DroneLauncher droneLauncher;
    GamepadEx gamepadEx1;
    Extender extender;

    private final double TRIGGER_THRESHOLD = 0.5;

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

        //This command needs to be tested.
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ServoTuningCommand(hardwareMap, "", telemetry, gamepadEx1));

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