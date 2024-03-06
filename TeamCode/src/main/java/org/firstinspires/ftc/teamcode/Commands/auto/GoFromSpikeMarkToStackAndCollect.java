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
import org.firstinspires.ftc.teamcode.Commands.utilCommands.DetectionSideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class GoFromSpikeMarkToStackAndCollect extends SequentialCommandGroup {
    public GoFromSpikeMarkToStackAndCollect(RobotControl robot) {
        addCommands(
                new ParallelCommandGroup(
                        getTrajectoryCommand(robot),
                        new WaitCommand(200).andThen(new ArmGetToPosition(robot, ArmPosition.INTAKE, false), new WaitCommand(300), new CartridgeSetState(robot.cartridge, Cartridge.State.INTAKE_OPEN)),
                        new InstantIntakeRotate(robot, robot.intake.roller.COLLECT_POWER),
                        new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL)
                ),
                new CollectFromStack(robot)
        );
    }


    private Command getTrajectoryCommand(RobotControl robot) {
        return new ConditionalCommand(
                new DetectionSideCommandSwitch(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Far Detected) Red"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Center Detected) Red"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Close Detected) Red"), robot.autoDriveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                new DetectionSideCommandSwitch(
                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Far Detected) Blue"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Center Detected) Blue"), robot.autoDriveTrain),
                        new TrajectoryFollowerCommand(robot.trajectories.get("Driving to stack (Close Detected) Blue"), robot.autoDriveTrain),
                        () -> robot.teamPropDetector.getTeamPropSide()
                ),
                () -> robot.allianceColor == AllianceColor.RED
        );
    }

}
