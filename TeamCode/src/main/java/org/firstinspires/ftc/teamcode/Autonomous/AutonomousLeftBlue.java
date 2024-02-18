package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

@Autonomous(name = "AutonomousLeftBlue")
public class AutonomousLeftBlue extends AutonomousLeft{
    public AutonomousLeftBlue() {
        super(AllianceColor.BLUE);
    }
}