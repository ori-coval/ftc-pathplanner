package org.firstinspires.ftc.teamcode.Utils;

public class Configuration {
    public static final String DRIVE_TRAIN_BACK_LEFT = "backLeft"; //0
    public static final String DRIVE_TRAIN_BACK_RIGHT = "backRight"; //3
    public static final String DRIVE_TRAIN_FRONT_LEFT = "frontLeft"; //1
    public static final String DRIVE_TRAIN_FRONT_RIGHT = "frontRight"; //2
    public static final String IMU = "imu";
    public static final String SHOOTER = "shooter";

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
                    4:
                    5:

            DIGITAL DEVICES:
                    0:      (Digital Device)                     (digitalChannel)

            I2C Bus 0:
                    imu                      (REV internal IMU (BNO055))          (BNO055IMU)



  - EXPANSION HUB -
            MOTORS:
                    0:
                    1:
                    2:
                    3:

            SERVOS:
                    0:
                    1:
                    2:
                    3:
                    4:
                    5:

            Digital Devices:
                    0:
                    2:
   */