package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class OnlyForTesting extends CommandBase {
    private InTake inTake;
    private double target = inTake.STACK_POSITION[5];

    // the first value to 5 piksel is 0.62
    // the second value to 4 piksel is 0.58
    // the third value to 3 piksel is 0.55
    // the fourth value to 2 piksel is 0.52
    // the fifth value to 1 piksel is 0.49
    // כשהאיסוף למעלה 0.67

    public OnlyForTesting(InTake inTake) {
        this.inTake = inTake;
        this.addRequirements(inTake);
    }

    @Override
    public void initialize() {

    }



    @Override
    public void end(boolean interrupted) {
        inTake.stop();
    }

}
