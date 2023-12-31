package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class IntakeRotate extends CommandBase {
    private InTake inTake;
    private final double power;

    public IntakeRotate(InTake inTake, double power){
        this.inTake = inTake;
        this.power = power;
        this.addRequirements(inTake);
    }

    @Override
    public void initialize() {
        inTake.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        inTake.stop();
    }

}



