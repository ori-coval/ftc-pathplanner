package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.trajectoryUtils.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.ResetPixelCount;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class ScoringCommand extends SequentialCommandGroup {
    public ScoringCommand(Command scoringCommand, Command secondScoringCommand, RobotControl robot) {
        addCommands(
                new ParallelCommandGroup(
                        getTrajectoryCommand(robot),
                        new IntakeRotate(robot.intake.roller, robot.intake.roller.EJECT_POWER).withTimeout(1500),
                        new WaitCommand(1700).andThen(new ArmGetToPosition(robot, ArmPosition.SCORING, robot.allianceColor == AllianceColor.RED)).andThen(scoringCommand)
                ),
                new WaitCommand(300),
                new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN),
                new ResetPixelCount(robot),
                new WaitCommand(300),
                secondScoringCommand
        );
    }


    private Command getTrajectoryCommand(RobotControl robot) {
        return new ConditionalCommand(
                new TrajectoryFollowerCommand(robot.trajectories.get("Go to backdrop (Far Side) Red"), robot.autoDriveTrain),
                new TrajectoryFollowerCommand(robot.trajectories.get("Go to backdrop (Far Side) Blue"), robot.autoDriveTrain),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

}
