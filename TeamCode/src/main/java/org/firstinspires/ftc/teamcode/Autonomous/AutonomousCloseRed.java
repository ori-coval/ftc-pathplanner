package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

@Autonomous(name = "AutonomousCloseRed")
public class AutonomousCloseRed extends AutonomousOpMode {
    public AutonomousCloseRed() {
        super(AllianceColor.RED, AllianceSide.CLOSE);
    }
}