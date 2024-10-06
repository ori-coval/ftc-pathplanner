package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;

public class ScoringArmSetState extends InstantCommand {


    public ScoringArmSetState(ScoringArm scoringArm, ScoringArm.Position position) {
        super(() -> scoringArm.setPosition(position.scoringArmPosition));


    }
}
