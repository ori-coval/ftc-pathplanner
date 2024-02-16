package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.auto.AutoTest;
import org.firstinspires.ftc.teamcode.Commands.auto.GoFromSpikeMarkToStackAndCollectPixel;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringPurplePixel;
import org.firstinspires.ftc.teamcode.Commands.auto.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.Side;


@Autonomous(name = "Autonomous")
public class AutonomousOpMode extends CommandOpMode {

    RobotControl robot;

    @Override
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, hardwareMap, gamepad1, gamepad2, telemetry);

        schedule(
                new SequentialCommandGroup(
                        new WaitCommand(0), //for some reason it runs the first command on the init
                        new ScoringPurplePixel(robot.autoDriveTrain, robot.intake, Side.CENTER, robot.elevator, robot.extender, robot.elbow, robot.turret, robot.antiTurret),
                        new GoFromSpikeMarkToStackAndCollectPixel(robot.autoDriveTrain, robot.intake, Side.CENTER, robot.elevator, robot.extender, robot.elbow, robot.turret, robot.antiTurret, robot.cartridge)
                )
        );
    }


}
