package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetLifterPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.InstantIntakeRotate;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class GoToStackForSecondCycleAndCollect extends SequentialCommandGroup {
    public GoToStackForSecondCycleAndCollect(RobotControl robot) {
        addCommands(
                new ParallelCommandGroup(
                        getTrajectoryCommandPart1(robot),
                        new ArmGetToPosition(robot, ArmPosition.SCORING_AUTO, robot.allianceColor == AllianceColor.RED).andThen(
                                new ArmGetToPosition(robot, ArmPosition.INTAKE, true)
                        ),
                        new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS)
                ),
                new ParallelCommandGroup(
                        getTrajectoryCommandPart2(robot),
                        new WaitCommand(300).andThen(new CartridgeSetState(robot.cartridge, Cartridge.State.INTAKE_OPEN)),
                        new WaitCommand(500).andThen(
                                new InstantIntakeRotate(robot, robot.intake.roller.COLLECT_POWER),
                                new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.THIRD_PIXEL)
                        )
                ),
                new CollectFromStack(robot, true)
        );
    }

    private Command getTrajectoryCommandPart1(RobotControl robot) {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(
                        robot.trajectories.get("Back to stack (Second Cycle) Part 1 Red"),
                        robot.autoDriveTrain
                ),
                new TrajectoryFollowerCommand(
                        robot.trajectories.get("Back to stack (Second Cycle) Part 1 Blue"),
                        robot.autoDriveTrain
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

    private Command getTrajectoryCommandPart2(RobotControl robot) {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(
                        robot.trajectories.get("Back to stack (Second Cycle) Part 2 Red"),
                        robot.autoDriveTrain
                ),
                new TrajectoryFollowerCommand(
                        robot.trajectories.get("Back to stack (Second Cycle) Part 2 Blue"),
                        robot.autoDriveTrain
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }


}
