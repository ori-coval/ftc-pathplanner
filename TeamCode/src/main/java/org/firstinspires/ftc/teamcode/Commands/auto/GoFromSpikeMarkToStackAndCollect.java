package org.firstinspires.ftc.teamcode.Commands.auto;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.BackToIntake;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class GoFromSpikeMarkToStackAndCollect extends SequentialCommandGroup {
    public GoFromSpikeMarkToStackAndCollect(RobotControl robot) {
        super(
                new ParallelCommandGroup(
                        new DetectionSideCommandSwitch(
                                new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Far Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Center Detected)"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Close Detected)"), robot.autoDriveTrain),
                                () -> robot.teamPropDetector.getTeamPropSide()
                        ),
                        new WaitCommand(200).andThen(new ArmGetToPosition(robot, ArmPosition.INTAKE, false).andThen(new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN))),
                        new InstantCommand(() -> robot.intake.roller.setPower(robot.intake.roller.COLLECT_POWER)),
                        new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL)
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new TrajectoryFollowerCommand(robot.trajectories.get("Drive back from stack"), robot.autoDriveTrain),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Drive back to stack"), robot.autoDriveTrain)
                        ),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(robot.intake.roller::isRobotFull).withTimeout(2500),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.intake.roller.stop()),
                                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED)
                        )
                )
        );
    }
}
