package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class OnlyForTesting extends CommandBase {
    private InTake inTake;
    public double position;

    public OnlyForTesting(InTake inTake, double position) {
        this.inTake = inTake;
        this.position = position;
        this.addRequirements(inTake);
    }

    // the first value to 5 piksel is 0.62
    // the second value to 4 piksel is 0.58
    // the third value to 3 piksel is 0.55
    // the fourth value to 2 piksel is 0.52
    // כשהאיסוף למעלה 0.67
    @Override
    public void initialize() {
        inTake.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        inTake.stop();
    }

}
