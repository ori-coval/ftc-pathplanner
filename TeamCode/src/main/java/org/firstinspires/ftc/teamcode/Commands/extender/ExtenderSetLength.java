package org.firstinspires.ftc.teamcode.Commands.extender;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Extender;

public class ExtenderSetLength extends InstantCommand {

    public ExtenderSetLength(Extender extender, double length){
        super(()->extender.setLength(length),extender);
    }
}
