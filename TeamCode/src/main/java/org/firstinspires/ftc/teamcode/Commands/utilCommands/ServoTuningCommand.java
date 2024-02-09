package org.firstinspires.ftc.teamcode.Commands.utilCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Configuration;
import org.firstinspires.ftc.teamcode.Utils.Servos;

public class ServoTuningCommand extends CommandBase {
    private Servo servo;
    private Servos selectedServo;
    private final Servo elbowLeft;
    Telemetry telemetry;
    GamepadEx gamepadEx1;
    HardwareMap hardwareMap;
    private boolean isOn;
    private double lastPos;
    private boolean isInitialized;

    public ServoTuningCommand(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx gamepadEx1) {
        this.telemetry = telemetry;
        this.gamepadEx1 = gamepadEx1;
        this.hardwareMap = hardwareMap;
        elbowLeft = hardwareMap.servo.get(Configuration.ELBOW_LEFT); //elbow has 2 servos
    }

    @Override
    public void initialize() {
        if(isInitialized) {
            lastPos = servo.getPosition();
            isOn = !isOn;
        } else {
            throw new RuntimeException("ServoTuningCommand hasn't been initialized yet. (using setServo)");
        }
    }

    public void setServo(Servos servoToTune) {
        selectedServo = servoToTune;
        servo = hardwareMap.servo.get(servoToTune.SERVO_NAME);
        isInitialized = true;
    }

    public String getServo() {
        return selectedServo.SERVO_NAME;
    }

    @Override
    public void execute() {
        if(servo == hardwareMap.servo.get(Servos.ELBOW.SERVO_NAME)) {
            elbowLeft.setPosition(lastPos + gamepadEx1.getLeftX() * (isOn ? 0.1 : 0));
        }

        servo.setPosition(lastPos + gamepadEx1.getLeftX() * (isOn ? 0.1 : 0));

        telemetry.addData("isOn", isOn);
        telemetry.addData("Servo Calculated Position", lastPos + gamepadEx1.getLeftX() * (isOn ? 0.1 : 0));
        telemetry.addData("Servo position", servo.getPosition());
        telemetry.addLine("-------------------");
        telemetry();
        telemetry.update();
    }


    public void telemetry() {

        telemetry.addLine("B - Anti Turret");
        telemetry.addLine("X - Cartridge");
        telemetry.addLine("D-Up - Elbow");
        telemetry.addLine("D-Down - Drone");
        telemetry.addLine("D-Right - Intake Lifter");
        telemetry.addLine("D-Left - Extender");

    }
}
