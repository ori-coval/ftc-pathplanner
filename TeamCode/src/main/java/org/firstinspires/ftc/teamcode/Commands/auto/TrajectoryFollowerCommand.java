package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

public class TrajectoryFollowerCommand extends CommandBase {

    TrajectorySequence trajectorySequence;
    SampleMecanumDrive driveTrain;

    public TrajectoryFollowerCommand(TrajectorySequence trajectorySequence, SampleMecanumDrive driveTrain) {
        this.trajectorySequence = trajectorySequence;
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        driveTrain.followTrajectorySequence(trajectorySequence);
    }

    @Override
    public boolean isFinished() {
        return !driveTrain.isBusy();
    }
}
