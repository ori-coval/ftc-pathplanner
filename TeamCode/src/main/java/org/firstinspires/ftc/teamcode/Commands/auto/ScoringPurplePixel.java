package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.conveyer.ConveyorConvey;
import org.firstinspires.ftc.teamcode.Commands.utils.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.Vision.Side;

public class ScoringPurplePixel extends SequentialCommandGroup {

    public ScoringPurplePixel(SampleMecanumDrive driveTrain, Conveyor conveyor, Side side) {
        super(
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Left"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Center"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Right"), driveTrain),
                        () -> side),
                new ConveyorConvey(conveyor, -0.5)
        );
    }
}
