package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

@Autonomous(name = "AutonomousFarRed")
public class AutonomousFarRed extends AutonomousOpMode {
    public AutonomousFarRed() {
        super(AllianceColor.RED, AllianceSide.FAR);
    }
}
