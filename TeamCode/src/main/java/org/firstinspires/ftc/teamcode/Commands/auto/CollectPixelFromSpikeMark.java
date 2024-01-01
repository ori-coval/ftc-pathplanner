package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.utils.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeSetPosition;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Conveyor;
import org.firstinspires.ftc.teamcode.SubSystems.InTake;
import org.firstinspires.ftc.teamcode.Vision.Side;

public class CollectPixelFromSpikeMark extends SequentialCommandGroup {
    public CollectPixelFromSpikeMark(SampleMecanumDrive driveTrain, InTake inTake, Conveyor conveyor, Side side) {
        super(
                new IntakeSetPosition(inTake, 3),
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Left"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Center"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Right"), driveTrain),
                        () -> side) ,
                new IntakeSetPosition(inTake, 2),
                new IntakeUntilFull(inTake, conveyor).withTimeout(10000)
        );
    }
}
