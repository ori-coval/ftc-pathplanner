/*
package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.BackToIntake;
import org.firstinspires.ftc.teamcode.Commands.auto.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.Side;

@Autonomous(name = "AutonomousLeftRed")
@Disabled
public class AutonomousLeftRed extends CommandOpMode {
    RobotControl robot;
    @Override
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, AllianceColor.RED, Side.LEFT, hardwareMap, gamepad1, gamepad2, telemetry);

        while(opModeInInit()) {
            if(robot.teamPropDetector.getTeamPropSide() != null) {
                schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(), //for some reason it runs the first command on the init
                                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED),
                                new InstantCommand(() -> robot.teamPropDetector.webcam.closeCameraDevice()),
                                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT),
                                new SideCommandSwitch(
                                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Left"), robot.driveTrain),
                                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Center"), robot.driveTrain),
                                        new TrajectoryFollowerCommand(Trajectories.get("Score Purple Right"), robot.driveTrain),
                                        () -> robot.teamPropDetector.getTeamPropSide()
                                ),
                                new ArmGetToPosition(robot, ArmPosition.AUTONOMOUS_PURPLE_PIXEL, false),
                                new IntakeRotate(robot.intake.roller, robot.intake.roller.COLLECT_POWER).withTimeout(3000),
                                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.FIRST_PIXEL),
                                new InstantCommand(() -> robot.intake.roller.setPower(robot.intake.roller.COLLECT_POWER)),
                                new SideCommandSwitch(
                                        new SequentialCommandGroup(
                                                new TrajectoryFollowerCommand(Trajectories.get("loading intake left"), robot.driveTrain),
                                                new BackToIntake(robot),
                                                new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Left"), robot.driveTrain)
                                        ),
                                        new SequentialCommandGroup(
                                                new TrajectoryFollowerCommand(Trajectories.get("loading intake center"), robot.driveTrain),
                                                new BackToIntake(robot),
                                                new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Center"), robot.driveTrain)
                                        ),
                                        new SequentialCommandGroup(
                                                new TrajectoryFollowerCommand(Trajectories.get("loading intake right"), robot.driveTrain),
                                                new BackToIntake(robot),
                                                new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Right"), robot.driveTrain)
                                        ),
                                        () -> robot.teamPropDetector.getTeamPropSide()
                                ),
*/
/*                                new TrajectoryFollowerCommand(Trajectories.get("Drive back from stack"), robot.driveTrain),
                                new TrajectoryFollowerCommand(Trajectories.get("Drive back to stack"), robot.driveTrain),
                                new TrajectoryFollowerCommand(Trajectories.get("Drive back from stack"), robot.driveTrain),
                                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.SECOND_PIXEL),
                                new TrajectoryFollowerCommand(Trajectories.get("Drive back to stack"), robot.driveTrain),
                                new TrajectoryFollowerCommand(Trajectories.get("Drive back from stack"), robot.driveTrain),*//*

                                new WaitCommand(2000),
                                new InstantCommand(() -> robot.intake.roller.stop()),
                                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED),
                                new TrajectoryFollowerCommand(Trajectories.get("Go to backdrop part 1"), robot.driveTrain),
                                new SideCommandSwitch(
                                        new ArmGetToPosition(robot, ArmPosition.SCORE_BOTTOM_CLOSE, true),
                                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_MID, true),
                                        new ArmGetToPosition(robot, ArmPosition.SCORE_AUTO_BOTTOM_FAR, true),
                                        () -> robot.teamPropDetector.getTeamPropSide()
                                ),
                                new TrajectoryFollowerCommand(Trajectories.get("Go to backdrop part 2"), robot.driveTrain),
                                new CartridgeSetState(robot.cartridge, Cartridge.State.OPEN),
                                new WaitCommand(500),
//                                new ElevatorGetToHeightPID(robot.elevator, 28),
//                                new InstantCommand(() -> ArmGetToPosition.lastPosition = ArmPosition.SCORE_TOP_CLOSE),
                                new ArmGetToPosition(robot, ArmPosition.SCORING, true),
                                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED),
                                new TrajectoryFollowerCommand(Trajectories.get("Go back after scoring yellow"), robot.driveTrain), //to allow intake to get in
                                new ArmGetToPosition(robot, ArmPosition.INTAKE, true)
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
*/
