package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class NormalTeleOpTest extends CommandOpMode {

    CRServo servo;

    @Override
    public void initialize() {
        servo = hardwareMap.get(CRServo.class, "servo");
    }

    @Override
    public void run() {
        super.run();
    }
}