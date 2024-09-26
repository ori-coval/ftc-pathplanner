package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;

public class IntakeArmSetState extends InstantCommand {

    public IntakeArmSetState(IntakeArm armAngle, IntakeArm.Position position){
        super(()-> armAngle.setPosition(position.intakeArmPosition));

    }
}
