package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.turret.RotateTurretByPower;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorIMUNonOrthogonal;
import org.firstinspires.ftc.teamcode.Commands.antiTurret.AntiTurretParallel;
import org.firstinspires.ftc.teamcode.Commands.elbow.ElbowGetToAngle;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeFromStack;
import org.firstinspires.ftc.teamcode.Commands.drivetrain.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.intake.TeleopIntake;
import org.firstinspires.ftc.teamcode.Commands.turret.RotateTurretByPower;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
import org.firstinspires.ftc.teamcode.SubSystems.Odometry;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.AllianceColor;
import org.firstinspires.ftc.teamcode.Vision.Side;
import org.firstinspires.ftc.teamcode.Vision.TeamPropDetector;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "DriveTrein")
public class OpMode extends CommandOpMode{
    DriveTrain driveTrain;
//    InTake inTake;
//    Elbow elbow;
//    Turret turret;
//    AntiTurret antiTurret;
    TeamPropDetector teamPropDetector;
    VisionPortal portal;
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

        teamPropDetector = new TeamPropDetector(AllianceColor.RED);
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Weiss cam"), teamPropDetector); //webcam 1

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(()-> odometry.resetLocation()));
    }

    @Override
    public void run() {
        super.run();

        if (opModeIsActive()) {
            telemetry.addData("LeftBlue", teamPropDetector.getSideColor(Side.LEFT,2));
            telemetry.addData("RightBlue", teamPropDetector.getSideColor(Side.RIGHT,2));
            telemetry.addData("CenterBlue", teamPropDetector.getSideColor(Side.CENTER,2));
            telemetry.addData("LeftRed", teamPropDetector.getSideColor(Side.LEFT,0));
            telemetry.addData("RightRed", teamPropDetector.getSideColor(Side.RIGHT,0));
            telemetry.addData("CenterRed", teamPropDetector.getSideColor(Side.CENTER,0));
            telemetry.update();
        }
        telemetry.addData("odometry", odometry.getLocation());
        telemetry.update();
    }
}
