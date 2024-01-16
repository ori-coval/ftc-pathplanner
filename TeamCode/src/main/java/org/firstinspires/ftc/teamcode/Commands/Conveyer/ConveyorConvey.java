package org.firstinspires.ftc.teamcode.Commands.Conveyer;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Conveyor;

public class ConveyorConvey extends CommandBase {
    private Conveyor conveyor;

    public double power;

    public ConveyorConvey(Conveyor conveyor, double power){
        this.power = power;
        this.conveyor = conveyor;
        this.addRequirements(conveyor);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {}
}
