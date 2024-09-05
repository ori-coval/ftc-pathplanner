package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class NormalTeleOpTest extends CommandOpMode {

    @Override
    public void initialize() {
        telemetry.addLine("HAII");
        telemetry.update();
    }
}
