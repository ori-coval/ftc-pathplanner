package org.firstinspires.ftc.teamcode.Commands.elbow;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Elbow;

public class ElbowGetToPositionTest extends InstantCommand {
    public ElbowGetToPositionTest(Elbow elbow, double position) {
        super(() -> elbow.setPosition(position)); //This works! but ofc it is too much power..
    }
}
