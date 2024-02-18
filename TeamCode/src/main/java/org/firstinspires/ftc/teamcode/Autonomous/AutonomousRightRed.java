package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

@Autonomous(name = "AutonomousRightRed")
public class AutonomousRightRed extends AutonomousRight {
    public AutonomousRightRed() {
        super(AllianceColor.RED);
    }
}