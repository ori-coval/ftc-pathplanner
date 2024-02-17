package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utils.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode.TrackingWheelForwardOffsetTuner;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class ScoringPurplePixel extends SequentialCommandGroup {

    private final long WAIT_UNTIL_EJECT_BACK = 2000;

    public ScoringPurplePixel(SampleMecanumDrive driveTrain, Intake intake, Side side, Elevator elevator, Extender extender, Elbow elbow, Turret turret, AntiTurret antiTurret) {
        addCommands(
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Left"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Center"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Right"), driveTrain),
                        () -> side
                ),
                new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.AUTONOMOUS_PURPLE_PIXEL_RIGHT, false),
                new IntakeRotate(intake.roller, intake.roller.COLLECT_POWER).withTimeout(WAIT_UNTIL_EJECT_BACK)
        );
    }
}
