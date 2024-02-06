package org.firstinspires.ftc.teamcode.Commands.auto;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.Commands.utils.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class GoCollectPixelFromSpikeMark extends SequentialCommandGroup {
    public GoCollectPixelFromSpikeMark(SampleMecanumDrive driveTrain, Intake intake, Side side) {
        super(
                new IntakeSetStackPosition(intake.lifter,3),
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Left"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Center"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Right"), driveTrain),
                        () -> side) ,
                new IntakeSetStackPosition(intake.lifter, 2),
                new IntakeUntilFull(intake.roller).withTimeout(10000)
        );
    }
}
