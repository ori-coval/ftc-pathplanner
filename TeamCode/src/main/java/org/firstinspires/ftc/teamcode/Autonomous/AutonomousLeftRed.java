package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

@Autonomous(name = "AutonmousLeftRed")
public class AutonomousLeftRed extends AutonomousLeft {
    public AutonomousLeftRed() {
        super(AllianceColor.RED);
    }
}