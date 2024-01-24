package org.firstinspires.ftc.teamcode.Commands.elbow;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Elbow;

public class ElbowGetToPosition extends InstantCommand {
    public ElbowGetToPosition(Elbow elbow, double goalAngle) {
        super(() -> elbow.setPosition(goalAngle), elbow);
    }
}
