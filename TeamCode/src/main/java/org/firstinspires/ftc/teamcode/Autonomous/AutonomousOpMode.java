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

        /* Not sure whether I need to run the scheduler, or when I need to do this, 'cause I can't make a loop here. I think. maybe there should be a loop of this running in the background. 'cause I would be using commands, and I do need to schedule them. RIGHT??
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
         */

        if(!isStopRequested() && opModeIsActive()) {
            new AutoTest(robot.autoDriveTrain).schedule(); //would this work without running the scheduler? no? right? I'm too tired to think, gn.
        }

        reset();
    }

}
