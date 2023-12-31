package org.firstinspires.ftc.teamcode.Commands.extender;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.checkerframework.checker.units.qual.Length;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;

public class ExtenderSetLength extends InstantCommand {

    public ExtenderSetLength(Extender extender, Extender.Length length){
        super(()->extender.setPosition(length),extender);
    }

}
