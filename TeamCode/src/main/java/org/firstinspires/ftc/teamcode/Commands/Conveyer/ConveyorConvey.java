package org.firstinspires.ftc.teamcode.Commands.Conveyer;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Conveyor;

public class ConveyorConvey extends CommandBase {
    private Conveyor conveyor;
    public ConveyorConvey(Conveyor conveyor){
        this.conveyor = conveyor;
        this.addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setPower(0.6);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }
}
