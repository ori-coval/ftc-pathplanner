package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.InitializeCommand;
import org.firstinspires.ftc.teamcode.Commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;


@Autonomous
@Disabled
public class AutoTest extends CommandOpMode {

    private RobotControl robot;


    @Override
    public void initialize() {

        robot = new RobotControl(RobotControl.OpModeType.AUTO, AllianceColor.RED, AllianceSide.FAR, hardwareMap, gamepad1, gamepad2, telemetry);

        SequentialCommandGroup command = new SequentialCommandGroup(
                new InitializeCommand(robot),
                new WaitUntilCommand(this::isStarted),
                new ParallelCommandGroup(
                        new TrajectoryFollowerCommand(robot.autoDriveTrain.trajectorySequenceBuilder(new Pose2d())
                                .forward(60)
                                .build(), robot.autoDriveTrain
                        ),
                        new WaitCommand(1000).andThen(new ElbowGetToPosition(new Elbow(hardwareMap, robot), 0.4))
                )
        );

        schedule(command);

    }
}
