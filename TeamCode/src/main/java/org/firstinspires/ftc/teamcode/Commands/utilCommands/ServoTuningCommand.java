package org.firstinspires.ftc.teamcode.Commands.utilCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoTuningCommand extends CommandBase {
    public static final double SENSITIVITY = 1;
    private double lastPos;
    Servo[] servos;
    Telemetry telemetry;
    GamepadEx gamepadEx1;
    public ServoTuningCommand(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx gamepadEx1, String... servosName) {
        servos = new Servo[servosName.length];
        this.telemetry = telemetry;
        this.gamepadEx1 = gamepadEx1;
        for (int i = 0; i < servosName.length; i++) {
            this.servos[i] = hardwareMap.servo.get(servosName[i]);
        }
    }

    @Override
    public void initialize() {
        lastPos = servos[0].getPosition();
    }

    @Override
    public void execute() {
        for (Servo servo : servos) {
            servo.setPosition(lastPos + gamepadEx1.getLeftX() * SENSITIVITY);
        }

        telemetry.addData("Servo Calculated Position", lastPos + gamepadEx1.getLeftX() * SENSITIVITY);
        telemetry.addData("Servo position", servos[0].getPosition());

        telemetry.addLine("----------------------");

        telemetry(telemetry);

    }

    public static void telemetry(Telemetry telemetry) {

        telemetry.addLine("B - Anti Turret");
        telemetry.addLine("Y - Drone");
        telemetry.addLine("X - Cartridge");
        telemetry.addLine("D-Up - Elbow");
        telemetry.addLine("D-Down - Drone");
        telemetry.addLine("D-Right - Intake Lifter");
        telemetry.addLine("D-Left - Extender");
//        telemetry.addLine("D-Left - Extender OPEN");
//        telemetry.addLine("L-Bumper - Extender SEMI-OPEN");
//        telemetry.addLine("R-Bumper - Extender CLOSED");

    }

}