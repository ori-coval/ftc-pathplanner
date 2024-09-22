package org.firstinspires.ftc.teamcode.Commands;

import android.icu.text.Transliterator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ArmAngle;

public class ArmAngleCommand extends InstantCommand {


    public ArmAngleCommand(ArmAngle armAngle,ArmAngle.Position position){
        super(()-> armAngle.setPosition(position.intakeArmPosition));
    }
}
