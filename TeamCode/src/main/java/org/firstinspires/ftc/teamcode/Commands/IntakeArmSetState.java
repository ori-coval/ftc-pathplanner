package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;

public class IntakeArmSetState extends InstantCommand {

    public IntakeArmSetState(IntakeArm.Position position){
        super(
                ()-> MMRobot.getInstance().mmSystems.intakeArm.setPosition(position.intakeArmPosition),
                MMRobot.getInstance().mmSystems.intakeArm
        );

    }
}
