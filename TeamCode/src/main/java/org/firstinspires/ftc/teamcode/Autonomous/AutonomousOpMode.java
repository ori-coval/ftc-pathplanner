package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public class AutonomousOpMode extends LinearOpMode {

    public static RobotControl robot;
    public AllianceColor allianceColor;
    
    public AutonomousOpMode(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, allianceColor, hardwareMap, gamepad1, gamepad2, telemetry);
        SequentialCommandGroup commandsToRun = null;



        commandsToRun = getCommandsToRun();
        robot.schedule(commandsToRun);
    }

    private SequentialCommandGroup getCommandsToRun() {
        SequentialCommandGroup result = new SequentialCommandGroup();

        return result;

    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            robot.run();


            FtcDashboard.getInstance().getTelemetry().update();
            telemetry.update();
        }


        RobotControl.lastHeading = robot.driveTrain.getYawInDegrees();
        robot.reset();
    }

}
