package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.antiTurret.AntiTurretParallel;
import org.firstinspires.ftc.teamcode.Commands.drivetrain.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.turret.RotateTurretByPower;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
import org.firstinspires.ftc.teamcode.SubSystems.Odometry;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.AllianceColor;
import org.firstinspires.ftc.teamcode.Vision.TeamPropDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TestOpMode extends CommandOpMode {

    DriveTrain driveTrain;
    BNO055IMU imu;
    Turret turret;
    GamepadEx gamepadEx1;
    double power;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        DriveTrainInit();
        IMUInit();
//        TurretInit();

        gamepadEx1 = new GamepadEx(gamepad1);
        power = gamepad1.left_trigger-gamepad1.right_trigger;
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> turret.setPower(power)));

    }
    public void DriveTrainInit() {
        IMUInit();
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("backLeftLin")
                , hardwareMap.dcMotor.get("backRightLin")
                , hardwareMap.dcMotor.get("frontRightLin")
                , hardwareMap.dcMotor.get("frontLeftLin")
                , imu);
        driveTrain.setDefaultCommand(new TeleopDriveCommand(driveTrain, gamepad1));
    }
    public void IMUInit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }
//    public void TurretInit() {
//        turret = new Turret(
//                hardwareMap.crservo.get("turretMotorA"),
//                hardwareMap.crservo.get("turretMotorB"),
////                new InTake()
//        );
//    }

    @Override
    public void run() {
        super.run();


    }
}
