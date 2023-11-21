package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.turret.RotateTurretByPower;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.AllianceColor;
import org.firstinspires.ftc.teamcode.Vision.Side;
import org.firstinspires.ftc.teamcode.Vision.TeamPropDetector;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "DriveTrein")
public class OpMode extends CommandOpMode {
    //    DriveTrain driveTrain;
//    InTake inTake;
//    Elbow elbow;
    TeamPropDetector teamPropDetector;
    VisionPortal portal;
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
//        turret = new Turret(
//                hardwareMap.crservo.get("turretMotorA"),
//                hardwareMap.crservo.get("turretMotorB"),
//                hardwareMap.analogInput.get("turretEncoder")
//        );

        teamPropDetector = new TeamPropDetector(AllianceColor.RED);
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Weiss cam"), teamPropDetector); //webcam 1

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileActiveOnce(new RotateTurretByPower(0.2, turret));
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
    }
}
