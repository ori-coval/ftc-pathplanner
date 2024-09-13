package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MMInitMethods;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

import java.util.logging.Handler;

@TeleOp
public class Test extends CommandOpMode {

    CRServo servo;
    @Override
    public void initialize() {
        servo = hardwareMap.crservo.get("n");
    }


    @Override
    public void run() {
        super.run();


        servo.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
    }
}
