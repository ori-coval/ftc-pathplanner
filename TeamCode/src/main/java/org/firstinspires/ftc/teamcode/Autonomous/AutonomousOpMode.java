package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.auto.AutoTest;
import org.firstinspires.ftc.teamcode.RobotControl;


@Autonomous(name = "Autonomous")
public class AutonomousOpMode extends CommandOpMode {

    RobotControl robot;

    @Override
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        new AutoTest(robot.autoDriveTrain).schedule(); //would this work without running the scheduler? no? right? I'm too tired to think, gn.

        reset();
    }

}
