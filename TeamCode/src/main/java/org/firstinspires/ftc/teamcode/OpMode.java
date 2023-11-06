package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorIMUNonOrthogonal;
import org.firstinspires.ftc.teamcode.Commands.elbow.ElbowGetToAngle;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeFromStack;
import org.firstinspires.ftc.teamcode.Commands.drivetrain.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.intake.TeleopIntake;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

@TeleOp(name = "DriveTrein")
public class OpMode extends CommandOpMode{
    DriveTrain driveTrain;

//    InTake inTake;
//    Elbow elbow;
//    Turret turret;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("motorBL")
                ,hardwareMap.dcMotor.get("motorBR")
                ,hardwareMap.dcMotor.get("motorFR")
                ,hardwareMap.dcMotor.get("motorFL")
                ,imu);
            driveTrain.setDefaultCommand(new TeleopDriveCommand(driveTrain,gamepad1));
//         inTake = new InTake(hardwareMap.dcMotor.get("inTake"),hardwareMap.servo.get("intakeAngel"));
//         elbow = new Elbow(hardwareMap.dcMotor.get("elbow"));
//        turret = new Turret(
//                hardwareMap.crservo.get("turretMotorA"),
//                hardwareMap.crservo.get("turretMotorB"),
//                hardwareMap.analogInput.get("turretEncoder")
//
//        );



//        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenActive(new IntakeFromStack(inTake));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenActive(new TeleopIntake(inTake));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(new TeleopIntake(inTake));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenActive(new InstantCommand(inTake::stop));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenActive(new ElbowGetToAngle(elbow, 0));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("before the normelize fr",driveTrain.calculationOfPowerRatio(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x)[0]);
        telemetry.addData("before the normelize fl",driveTrain.calculationOfPowerRatio(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x)[1]);
        telemetry.addData("before the normelize br",driveTrain.calculationOfPowerRatio(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x)[2]);
        telemetry.addData("before the normelize bl",driveTrain.calculationOfPowerRatio(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x)[3]);
        telemetry.addData("and after", DriveTrain.normalize(driveTrain.calculationOfPowerRatio(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x))[0]);
        telemetry.addData("and after", DriveTrain.normalize(driveTrain.calculationOfPowerRatio(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x))[1]);
        telemetry.addData("and after", DriveTrain.normalize(driveTrain.calculationOfPowerRatio(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x))[2]);
        telemetry.addData("and after", DriveTrain.normalize(driveTrain.calculationOfPowerRatio(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x))[3]);
        telemetry.addData("and after",driveTrain.getYaw());
        telemetry.update();
    }
}
