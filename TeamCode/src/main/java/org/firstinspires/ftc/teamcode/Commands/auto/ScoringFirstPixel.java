package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.utils.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ScoringFirstPixel extends SequentialCommandGroup {
    public ScoringFirstPixel(Elevator elevator, Extender extender, Elbow elbow, Turret turret, AntiTurret antiTurret, Side side){
        super(
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to score from left while avoiding pixel on Left"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to score from left while avoiding pixel on Center"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to score from right while avoiding pixel on Right"), driveTrain),
                        () -> side),
                new SideCommandSwitch(
                        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORE_BOTTOM_CLOSE, true),
                        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORE_BOTTOM_FAR, true),
                        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORE_BOTTOM_CLOSE, false),
                        () -> side),
                new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.INTAKE, true)

        );
    }

}
