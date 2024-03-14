package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.utilCommands.ServoTuningCommand;
import org.firstinspires.ftc.teamcode.RobotControl;


@TeleOp(name = "TuningOpMode")
public class TuningOpMode extends CommandOpMode {
    RobotControl robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new RobotControl(RobotControl.OpModeType.DEBUG, hardwareMap, gamepad1 ,gamepad2, telemetry);
    }


    @Override
    public void run() {
        super.run();

        ServoTuningCommand.telemetry(telemetry);

        telemetry.addLine("----------------------------");

        telemetry.addData("antiTurret pos", robot.antiTurret.getPosition());
        telemetry.addData("cartridge pos", robot.cartridge.getPosition());
        telemetry.addData("elbow pos", robot.elbow.getServoPosition());
        telemetry.addData("elevator height", robot.elevator.getHeight());
        telemetry.addData("turret angle", robot.turret.getAngle());
        telemetry.addData("Extender Servo Pos", robot.extender.getServoPosition());
        telemetry.addData("Extender Enum Pos", robot.extender.getExtenderPosition());
        telemetry.addLine("Remember that in the extender case,\nthere are CLOSED, MID_WAY and OPEN.\n" +
                "If you want to change them, you need to change it in Extender.java\n(or contact me if you want it differently)");
        telemetry.update();
    }
}