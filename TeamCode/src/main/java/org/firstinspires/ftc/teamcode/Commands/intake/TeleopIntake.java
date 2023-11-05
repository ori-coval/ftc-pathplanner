package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.InTake;

public class TeleopIntake extends CommandBase {
    double floorIntakePosition = 0;
    InTake inTake;

    public TeleopIntake(InTake inTake){
        this.inTake = inTake;
        this.addRequirements(inTake);
    }

    @Override
    public void initialize() {
        inTake.setPosition(floorIntakePosition);
        inTake.setPower(0.9);
    }

    @Override
    public void end(boolean interrupted) {
        inTake.stop();
    }

}



