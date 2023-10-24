package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ElbowGetToAngle;
import org.firstinspires.ftc.teamcode.Commands.IntakeFromStack;
import org.firstinspires.ftc.teamcode.Commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.TeleopIntake;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;

@TeleOp(name = "DriveTrein")
public class OpMode extends CommandOpMode{
    DriveTrain driveTrain;
    InTake inTake;
    Elbow elbow;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("motorBL")
                ,hardwareMap.dcMotor.get("motorBR")
                ,hardwareMap.dcMotor.get("motorFR")
                ,hardwareMap.dcMotor.get("motorFL"));
            driveTrain.setDefaultCommand(new TeleopDriveCommand(driveTrain,gamepad1));
         inTake = new InTake(hardwareMap.dcMotor.get("inTake"),hardwareMap.servo.get("intakeAngel"));
         elbow = new Elbow(hardwareMap.dcMotor.get("elbow"));


        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenActive(new IntakeFromStack(inTake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenActive(new TeleopIntake(inTake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(new TeleopIntake(inTake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenActive(new InstantCommand(inTake::stop));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenActive(new ElbowGetToAngle(elbow, 0));
    }
}
