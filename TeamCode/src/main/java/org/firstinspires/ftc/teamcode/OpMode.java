package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.drivetrain.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Odometry;

@TeleOp(name = "DriveTrein")
public class OpMode extends CommandOpMode{
    DriveTrain driveTrain;
//    InTake inTake;
//    Elbow elbow;
//    Turret turret;
//    AntiTurret antiTurret;
    Odometry odometry;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);


        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("backLeftLin")
                ,hardwareMap.dcMotor.get("backRightLin")
                ,hardwareMap.dcMotor.get("frontRightLin")
                ,hardwareMap.dcMotor.get("frontLeftLin")
                ,imu);
            driveTrain.setDefaultCommand(new TeleopDriveCommand(driveTrain,gamepad1));
//         inTake = new InTake(hardwareMap.dcMotor.get("inTake"),hardwareMap.servo.get("intakeAngel"));
//         elbow = new Elbow(hardwareMap.dcMotor.get("elbow"));
//         turret = new Turret(
//                hardwareMap.crservo.get("turretMotorA"),
//                hardwareMap.crservo.get("turretMotorB"),
//                hardwareMap.dcMotor.get("frontLeftLin")
//      );
        odometry = new Odometry(
                hardwareMap.dcMotor.get("frontLeftLin"),
                hardwareMap.dcMotor.get("backLeftLin")
        );
//        antiTurret = new AntiTurret(hardwareMap.servo.get("antiTurret"));
//        antiTurret.setDefaultCommand(new AntiTurretParallel(antiTurret, ()-> turret.getEncoderValue()));




        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(()-> odometry.resetLocation()));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("odometry", odometry.getLocation());
        telemetry.update();
    }
}
