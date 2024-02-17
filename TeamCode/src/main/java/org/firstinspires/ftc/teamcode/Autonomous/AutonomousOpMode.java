package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.Commands.auto.GoFromSpikeMarkToStack;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringPurplePixel;
import org.firstinspires.ftc.teamcode.Commands.auto.TakingFirstPixelFromStack;
import org.firstinspires.ftc.teamcode.Commands.auto.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeUntilFull;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.Side;


@Autonomous(name = "Autonomous")
public class AutonomousOpMode extends CommandOpMode {

    RobotControl robot;

    @Override
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, hardwareMap, gamepad1, gamepad2, telemetry);

        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(), //for some reason it runs the first command on the init
                        new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED),
                        new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT),
                        new ScoringPurplePixel(robot.autoDriveTrain, robot.intake, Side.RIGHT, robot.elevator, robot.extender, robot.elbow, robot.turret, robot.antiTurret),
                        new GoFromSpikeMarkToStack(robot.autoDriveTrain, robot.intake, Side.RIGHT, robot.elevator, robot.extender, robot.elbow, robot.turret, robot.antiTurret, robot.cartridge),
                        new TakingFirstPixelFromStack(robot),
                        new InstantCommand(() -> robot.intake.roller.stop())
                )
        );
    }


    @Override
    public void run() {
        super.run();
        telemetry.addData("pixel Count", robot.intake.roller.getPixelCount());
        telemetry.addData("isRobotFull", robot.intake.roller.isRobotFull());
        telemetry.update();
    }
}
