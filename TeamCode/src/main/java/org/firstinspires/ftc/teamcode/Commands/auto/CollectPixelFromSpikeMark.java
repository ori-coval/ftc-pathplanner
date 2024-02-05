package org.firstinspires.ftc.teamcode.Commands.auto;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.utils.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.Commands.intake.IntakeSetPosition;
import org.firstinspires.ftc.teamcode.Commands.multiSystem.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class CollectPixelFromSpikeMark extends SequentialCommandGroup {
    public CollectPixelFromSpikeMark(SampleMecanumDrive driveTrain, Intake intake, Side side) {
        super(
                new IntakeSetPosition(intake,3),
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Left"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Center"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Right"), driveTrain),
                        () -> side) ,
                new IntakeSetPosition(intake, 2),
                new IntakeUntilFull(intake).withTimeout(10000)
        );
    }
}
