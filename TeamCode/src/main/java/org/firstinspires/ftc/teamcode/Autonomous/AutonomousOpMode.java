package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.GoFromSpikeMarkToStackAndCollect;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringFirstPixel;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringPurplePixel;
import org.firstinspires.ftc.teamcode.Commands.auto.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;


@Autonomous(name = "Autonomous")
public class AutonomousOpMode extends CommandOpMode {

    RobotControl robot;

    @Override
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, hardwareMap, gamepad1, gamepad2, telemetry);

        while(!opModeIsActive()) {

            if(robot.teamPropDetector.getTeamPropSide() != null) {
                schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(), //for some reason it runs the first command on the init
                                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED),
                                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT),
                                new ScoringPurplePixel(robot),
                                new GoFromSpikeMarkToStackAndCollect(robot),
                                new ScoringFirstPixel(robot)
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
