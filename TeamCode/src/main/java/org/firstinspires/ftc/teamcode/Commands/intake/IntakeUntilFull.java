package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
public class IntakeUntilFull extends CommandBase {
    private InTake intake;

    public IntakeUntilFull(InTake intake) {
        this.intake = intake;
        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.isRobotFull(); {
            intake.stop();
        }

    }
}
