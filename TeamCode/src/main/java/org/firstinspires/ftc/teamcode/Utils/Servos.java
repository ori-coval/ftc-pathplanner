package org.firstinspires.ftc.teamcode.Utils;

public enum Servos {
    ELBOW(Configuration.ELBOW_RIGHT),
    EXTENDER(Configuration.EXTENDER),
    ANTI_TURRET(Configuration.ANTI_TURRET),
    CARTRIDGE(Configuration.CARTRIDGE),
    DRONE(Configuration.DRONE),
    INTAKE_LIFTER(Configuration.INTAKE_SERVO);

    public final String SERVO_NAME;

    Servos(String servoName) {
        SERVO_NAME = servoName;
    }
}
