package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("motorBL")
                ,hardwareMap.dcMotor.get("motorBR")
                ,hardwareMap.dcMotor.get("motorFR")
                ,hardwareMap.dcMotor.get("motorFL"));
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
        telemetry.addData("left x",gamepad1.left_stick_x);
        telemetry.addData("left y",gamepad1.left_stick_y);
        telemetry.addData("turn",gamepad1.right_stick_x);
        telemetry.update();
    }
}
