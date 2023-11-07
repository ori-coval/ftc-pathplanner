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
import org.firstinspires.ftc.teamcode.Commands.turret.RotateTurretByPower;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

@TeleOp(name = "DriveTrein")
public class OpMode extends CommandOpMode{
//    DriveTrain driveTrain;
//    InTake inTake;
//    Elbow elbow;
    Turret turret;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
//        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("motorBL")
//                ,hardwareMap.dcMotor.get("motorBR")
//                ,hardwareMap.dcMotor.get("motorFR")
//                ,hardwareMap.dcMotor.get("motorFL"));
//            driveTrain.setDefaultCommand(new TeleopDriveCommand(driveTrain,gamepad1));
//         inTake = new InTake(hardwareMap.dcMotor.get("inTake"),hardwareMap.servo.get("intakeAngel"));
//         elbow = new Elbow(hardwareMap.dcMotor.get("elbow"));
        turret = new Turret(
                hardwareMap.crservo.get("turretMotorA"),
                hardwareMap.crservo.get("turretMotorB"),
                hardwareMap.analogInput.get("turretEncoder")
        );



        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileActiveOnce(new RotateTurretByPower(0.2, turret));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("encoder value",turret.getEncoderValue());
        telemetry.update();
    }
}
