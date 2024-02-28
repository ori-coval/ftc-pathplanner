package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

@Autonomous(name = "AutonomousCloseBlue")
public class AutonomousCloseBlue extends AutonomousOpMode {
    public AutonomousCloseBlue() {
        super(AllianceColor.BLUE, AllianceSide.CLOSE);
    }
}