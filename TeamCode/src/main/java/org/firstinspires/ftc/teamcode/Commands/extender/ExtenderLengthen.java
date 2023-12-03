package org.firstinspires.ftc.teamcode.Commands.extender;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Extender;

public class ExtenderLengthen extends CommandBase {
    private Extender extender;
    private final double position0 = 0;
    private final double position1 = 0;
    private final double position2 =0;
    public ExtenderLengthen(Extender extender){
        this.extender = extender;
        addRequirements(extender);
    }

    public void setPosition(double position){
        extender.setPosition(position);
    }
}
