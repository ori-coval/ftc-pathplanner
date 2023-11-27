package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeRotate extends CommandBase {
    InTake inTake;

    public IntakeRotate(InTake inTake){
        this.inTake = inTake;
        this.addRequirements(inTake);
    }

    @Override
    public void initialize() {
        inTake.setPower(0.7);
    }

    @Override
    public void end(boolean interrupted) {
        inTake.stop();
    }

}



