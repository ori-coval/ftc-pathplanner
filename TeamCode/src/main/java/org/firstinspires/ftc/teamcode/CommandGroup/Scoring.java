package org.firstinspires.ftc.teamcode.CommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.ClawSetState;
import org.firstinspires.ftc.teamcode.Commands.ScoringArmSetState;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;

public class Scoring extends SequentialCommandGroup {
    public Scoring(Elevator elevator, ScoringArm scoringArm, Claw claw,double high){
        addCommands(
                new ClawSetState(MMRobot.getInstance().mmSystems.claw,Claw.State.CLOSE),
                new MMPIDCommand(MMRobot.getInstance().mmSystems.elevator,high),
                new ScoringArmSetState(scoringArm,ScoringArm.Position.SCORING)
        );
        addRequirements(elevator,scoringArm,claw);
    }
}
