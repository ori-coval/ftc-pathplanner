package org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.AutoDriveTrain;

public class TrajectoryFollowerCommand extends CommandBase {

    private final AutoDriveTrain drive;
    private final TrajectorySequence trajectorySequence;

    public TrajectoryFollowerCommand(TrajectorySequence trajectorySequence, AutoDriveTrain drive) {
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequence(trajectorySequence);
    }

    @Override
    public void execute() {
        drive.update();
    }


    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}
