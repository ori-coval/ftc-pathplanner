package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

@Autonomous(name = "AutonomousFarBlue")
public class AutonomousFarBlue extends AutonomousOpMode {
    public AutonomousFarBlue() {
        super(AllianceColor.BLUE, AllianceSide.FAR);
    }
}
