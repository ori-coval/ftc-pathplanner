package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

public class AutoTest extends CommandOpMode {

    private DriveTrain driveTrain;

    @Override
    public void initialize() {

        driveTrain = new DriveTrain(new SampleMecanumDrive(hardwareMap), false);

        SequentialCommandGroup command = new SequentialCommandGroup(
                new WaitUntilCommand(this::isStarted),
                new TrajectoryFollowerCommand(driveTrain.trajectorySequenceBuilder(new Pose2d())
                        .forward(30)
                        .build(), driveTrain
                )
        );

        /*TODO: Things to check in case this doesn't work
        *  TrajectoryFollorCommand gets followSequenceTrajectory and not followSequenceTrajectoryAsync
        *  compare with StraightTest that should 100% work.
        */

        schedule(command);

    }
}
