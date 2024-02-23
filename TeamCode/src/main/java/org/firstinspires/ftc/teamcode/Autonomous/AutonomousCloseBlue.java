package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.Side;

@Autonomous(name = "AutonomousCloseBlue")
@Disabled
public class AutonomousCloseBlue extends CommandOpMode {

    RobotControl robot;

    @Override
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, AllianceColor.BLUE, AllianceSide.CLOSE, hardwareMap, gamepad1, gamepad2, telemetry);

        while (opModeInInit()) {
            if (robot.teamPropDetector.getTeamPropSide() != null) {
                schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(), //for some reason it runs the first command on the init
                                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED),
                                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT),
                                new TrajectoryFollowerCommand(robot.trajectories.get("Score Purple Center RightBlue"), robot.driveTrain),
                                new ArmGetToPosition(robot, ArmPosition.AUTONOMOUS_PURPLE_PIXEL, false),
                                new IntakeRotate(robot.intake.roller, robot.intake.roller.COLLECT_POWER).withTimeout(3000)
                        )
                );
            }
            robot.teamPropDetector.telemetry();
        }
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}