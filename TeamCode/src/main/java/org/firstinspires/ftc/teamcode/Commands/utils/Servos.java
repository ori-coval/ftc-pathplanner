package org.firstinspires.ftc.teamcode.Commands.utils;

import com.qualcomm.robotcore.hardware.Servo;

public enum Servos {
    ELBOW("elbowRight"),
    EXTENDER("extender"),
    ANTI_TURRET("antiTurret"),
    CARTRIDGE("cartridge"),
    DRONE("drone"),
    INTAKE_LIFTER("intakeServo");

    final String servoName;

    Servos(String servoName) {
        this.servoName = servoName;
    }
}
