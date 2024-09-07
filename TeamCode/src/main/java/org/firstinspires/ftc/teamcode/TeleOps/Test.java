package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MMInitMethods;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

import java.util.logging.Handler;

@TeleOp
public class Test extends CommandOpMode {
    @Override
    public void initialize() {
        MMRobot.getInstance().init(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION, hardwareMap, gamepad1,gamepad2,telemetry);
        MMInitMethods.initDriveTrain();
    }


    @Override
    public void run() {
        super.run();
    }
}
