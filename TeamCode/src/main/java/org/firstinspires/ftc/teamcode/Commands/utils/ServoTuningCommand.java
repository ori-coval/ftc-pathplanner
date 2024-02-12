package org.firstinspires.ftc.teamcode.Commands.utils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoTuningCommand extends CommandBase {
    public static final double SENSITIVITY = 0.1;
    private double lastPos;
    Servo servo;
    Telemetry telemetry;
    GamepadEx gamepadEx1;
    public ServoTuningCommand(HardwareMap hardwareMap, String servoName, Telemetry telemetry, GamepadEx gamepadEx1) {
        this.servo = hardwareMap.servo.get(servoName);
        this.telemetry = telemetry;
        this.gamepadEx1 = gamepadEx1;
    }

    @Override
    public void initialize() {
        lastPos = servo.getPosition();
    }

    @Override
    public void execute() {
        servo.setPosition(lastPos + gamepadEx1.getLeftX() * SENSITIVITY);

        telemetry.addData("Servo Calculated Position", lastPos + gamepadEx1.getLeftX() * SENSITIVITY);
        telemetry.addData("Servo position", servo.getPosition());
    }
}
