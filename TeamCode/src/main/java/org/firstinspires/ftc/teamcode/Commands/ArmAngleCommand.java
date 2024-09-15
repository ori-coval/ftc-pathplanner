package org.firstinspires.ftc.teamcode.Commands;

import android.icu.text.Transliterator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ArmAngle;

public class ArmAngleCommand extends CommandBase {

    ArmAngle armAngle = MMRobot.getInstance().mmSystems.armAngle;

    ArmAngleCommand(Enum p){

    }
}
