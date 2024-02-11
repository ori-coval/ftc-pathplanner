package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utils.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class ScoringPurplePixel extends SequentialCommandGroup {

    public ScoringPurplePixel(SampleMecanumDrive driveTrain, Intake intake, Side side, Extender extender, Elbow elbow) {
        super(

                new ExtenderSetPosition(extender, Extender.Position.CLOSED),
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Left"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Center"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Right"), driveTrain),
                        () -> side),
                new ElbowGetToPosition(elbow, 0),
                new IntakeRotate(intake.roller, -1)
        );
    }
}
