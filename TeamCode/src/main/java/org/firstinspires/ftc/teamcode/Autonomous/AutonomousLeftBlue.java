package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.BackToIntake;
import org.firstinspires.ftc.teamcode.Commands.auto.AutoInit;
import org.firstinspires.ftc.teamcode.Commands.auto.GoFromSpikeMarkToStackAndCollect;
import org.firstinspires.ftc.teamcode.Commands.auto.ParkingAfterScoringYellow;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringFirstPixelAuto;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringPurplePixel;
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

@Autonomous(name = "AutonomousLeftBlue")
public class AutonomousLeftBlue extends CommandOpMode {
    RobotControl robot;
    @Override
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, AllianceColor.BLUE, Side.LEFT, hardwareMap, gamepad1, gamepad2, telemetry);

        while(opModeInInit()) {
            if(robot.teamPropDetector.getTeamPropSide() != null) {
                schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(), //for some reason it runs the first command on the init
                                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED),
                                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT),
                                new TrajectoryFollowerCommand(Trajectories.get("Score Purple Center LeftBlue"), robot.autoDriveTrain),
                                new ArmGetToPosition(robot, ArmPosition.AUTONOMOUS_PURPLE_PIXEL_RIGHT, false),
                                new IntakeRotate(robot.intake.roller, robot.intake.roller.COLLECT_POWER).withTimeout(3000)
//                                new TrajectoryFollowerCommand(Trajectories.get("Park on LeftBlue"), robot.autoDriveTrain)
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
