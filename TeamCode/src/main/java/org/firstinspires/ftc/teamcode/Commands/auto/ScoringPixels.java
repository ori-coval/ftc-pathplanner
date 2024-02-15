package org.firstinspires.ftc.teamcode.Commands.auto;

import android.app.Notification;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeCollectFromStack;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeUntilFullAndEject;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.utils.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ScoringPixels extends SequentialCommandGroup {
            public ScoringPixels(SampleMecanumDrive driveTrain, Intake intake, Elevator elevator, Extender extender, Elbow elbow, Turret turret, AntiTurret antiTurret, Cartridge cartridge, Side side) {
                super(
                        new ParallelCommandGroup(
                                new TrajectoryFollowerCommand(Trajectories.get("Driving from board to collect from stack"), driveTrain),
                                new IntakeSetStackPosition(intake.lifter, Intake.LifterPosition.STANDBY),
                                new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.INTAKE, true)
                        ),
                        new IntakeCollectFromStack(intake.lifter, intake.roller).withTimeout(0),
                        new IntakeSetStackPosition(intake.lifter, Intake.LifterPosition.STANDBY),
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to score from pixel stack"), driveTrain),
                        new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.SCORE_BOTTOM_CLOSE, true)

                        );
            }
}
