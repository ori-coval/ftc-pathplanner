package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Conveyer.ConveyorConvey;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
import org.firstinspires.ftc.teamcode.Vision.Side;

import java.util.HashMap;

public class ScoringPurplePixel extends SequentialCommandGroup {

    public ScoringPurplePixel(SampleMecanumDrive driveTrain, Conveyor conveyor, Side side){
        super(
                new SelectCommand(new HashMap<Object, Command>() {{
                    put(Side.RIGHT, new TrajectoryFollowerCommand(Trajectories.get("Score Purple Right"), driveTrain));
                    put(Side.CENTER, new TrajectoryFollowerCommand(Trajectories.get("Scoring Purple Center"), driveTrain));
                    put(Side.LEFT, new TrajectoryFollowerCommand(Trajectories.get("Scoring Purple Left"), driveTrain));
                }}, () -> side),
                new ConveyorConvey(conveyor, -0.5)
                );

    }

}
