package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

@Autonomous(name = "AutonomousCloseBlue")
public class AutonomousCloseBlue extends AutonomousClose {
    public AutonomousCloseBlue() {
        super(AllianceColor.BLUE);
    }
}