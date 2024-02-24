package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

@Autonomous(name = "AutonomousCloseRed")
public class AutonomousCloseRed extends AutonomousClose {
    public AutonomousCloseRed() {
        super(AllianceColor.RED);
    }
}