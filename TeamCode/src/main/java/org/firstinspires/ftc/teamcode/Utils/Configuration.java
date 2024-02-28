package org.firstinspires.ftc.teamcode.Utils;

public class Configuration {
    public static final String DRIVE_TRAIN_BACK_LEFT = "backLeft"; //0
    public static final String DRIVE_TRAIN_BACK_RIGHT = "backRight"; //3
    public static final String DRIVE_TRAIN_FRONT_LEFT = "frontLeft"; //1
    public static final String DRIVE_TRAIN_FRONT_RIGHT = "frontRight"; //2
    public static final String IMU = "imu";
    public static final String TURRET = "turret";
    public static final String TURRET_ENCODER = DRIVE_TRAIN_FRONT_RIGHT;
    public static final String INTAKE_SERVO = "intakeServo";
    public static final String INTAKE_MOTOR = "intake";
    public static final String INTAKE_SWITCH = "switch";
    public static final String ELEVATOR_RIGHT = "elevatorRight";
    public static final String ELEVATOR_LEFT = "elevatorLeft";
    public static final String ELEVATOR_CLIMBER = "elevatorClimber";
    public static final String ELBOW_RIGHT = "elbowRight";
    public static final String ELBOW_LEFT = "elbowLeft";
    public static final String CARTRIDGE = "cartridge";
    public static final String ANTI_TURRET = "antiTurret";
    public static final String EXTENDER = "extender";
    public static final String DRONE = "drone";


    public static final String WEB_CAM = "Weiss cam";

}


/*
                      - NAME -                  - TYPE IN CONFIG -             - TYPE IN CODE -

  - CONTROL HUB -
            MOTORS:
                    0: backLeft              (GoBilda 5202/3/4 series)            (dcMotor)
                    1: frontLeft             (GoBilda 5202/3/4 series)            (dcMotor)
                    2: frontRight            (GoBilda 5202/3/4 series)            (dcMotor)
                    3: backRight             (GoBilda 5202/3/4 series)            (dcMotor)

            SERVOS:
                    0:
                    1:
                    2:
                    3:
                    4: drone                 (servo)                              (servo)
                    5: intakeServo           (servo)                              (servo)

            DIGITAL DEVICES:
                    0: conveyorSwitch       (Digital Device)                     (digitalChannel)

            I2C Bus 0:
                    imu                      (REV internal IMU (BNO055))          (BNO055IMU)



  - EXPANSION HUB -
            MOTORS:
                    0: elevatorLeft            (GoBilda 5202/3/4 series)            (dcMotor)
                    1: elevatorRight           (GoBilda 5202/3/4 series)            (dcMotor)
                    2: turret                  (GoBilda 5202/3/4 series)            (dcMotor)
                    3: intake                  (GoBilda 5202/3/4 series)            (dcMotor)

            SERVOS:
                    0: elbowRight            (servo)                              (servo)
                    1: extender              (servo)                              (servo)
                    2: elbowLeft             (servo)                              (servo)
                    3:                       (servo)                              (servo)
                    4: cartridge             (servo)                              (servo)
                    5: antiTurret            (servo)                              (servo)

            Digital Devices:
                    0: safePlaceSwitch
                    2: elevatorSwitch
   */