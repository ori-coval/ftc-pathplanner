package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeCollectFromStack;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ScoringPixels extends SequentialCommandGroup {
            public ScoringPixels(RobotControl robot, Side side) {
                super(
                        new ParallelCommandGroup(
                                new TrajectoryFollowerCommand(robot.trajectories.get("Driving from board to collect from stack"), robot.autoDriveTrain),
                                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.STANDBY),
                                new ArmGetToPosition(robot, ArmPosition.INTAKE, true)
                        ),
                        new IntakeCollectFromStack(robot.intake.lifter, robot.intake.roller).withTimeout(0),
                        new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.STANDBY),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to score from pixel stack"), robot.autoDriveTrain),
                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE, true)

                );
            }
}
