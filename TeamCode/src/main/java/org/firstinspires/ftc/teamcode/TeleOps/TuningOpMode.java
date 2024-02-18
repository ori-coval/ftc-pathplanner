package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.utilCommands.ServoTuningCommand;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.Side;


@TeleOp(name = "TuningOpMode")
public class TuningOpMode extends CommandOpMode {
    RobotControl robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new RobotControl(RobotControl.OpModeType.DEBUG, AllianceColor.RED, Side.RIGHT, hardwareMap, gamepad1 ,gamepad2, telemetry);
    }


    @Override
    public void run() {
        super.run();

        ServoTuningCommand.telemetry(telemetry);

        telemetry.addLine("----------------------------");

        telemetry.addData("antiTurret pos", robot.antiTurret.getPosition());
        telemetry.addData("elbow", robot.elbow.getServoPosition());
        telemetry.addData("elevator height", robot.elevator.getHeight());
        telemetry.addData("turret angle", robot.turret.getAngle());
        telemetry.addData("Extender pos", robot.extender.getPosition());

        telemetry.update();
    }
}