package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.ScoringArmSetState;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;

public class ElevatorBackTo_0 extends ParallelCommandGroup {

    public ElevatorBackTo_0(){
        super(
                new MMPIDCommand(MMRobot.getInstance().mmSystems.elevator,1),
                new ScoringArmSetState(MMRobot.getInstance().mmSystems.scoringArm,ScoringArm.Position.IN)
        );
        addRequirements(
                MMRobot.getInstance().mmSystems.elevator,
                MMRobot.getInstance().mmSystems.scoringArm
        );
    }

}
