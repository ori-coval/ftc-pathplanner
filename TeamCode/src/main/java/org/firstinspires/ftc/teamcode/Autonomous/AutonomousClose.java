package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.auto.AutoInit;
import org.firstinspires.ftc.teamcode.Commands.auto.ParkingAfterScoringYellow;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoreYellowClose;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringPurplePixel;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

public class AutonomousClose extends LinearOpMode {

    RobotControl robot;

    public AllianceColor allianceColor;

    public AutonomousClose(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, allianceColor, AllianceSide.CLOSE, hardwareMap, gamepad1, gamepad2, telemetry);

        while(opModeInInit() && !isStopRequested()) {
            if(robot.teamPropDetector.getTeamPropSide() != null) {
                SequentialCommandGroup commandsToRun = new SequentialCommandGroup(
                        new WaitUntilCommand(this::isStarted),
                        new AutoInit(robot),
                        new ScoringPurplePixel(robot),
                        new ScoreYellowClose(robot),
                        new ParkingAfterScoringYellow(robot)
                );

                robot.schedule(commandsToRun);
            }
            robot.teamPropDetector.telemetry();
        }
        robot.teamPropDetector.webcam.closeCameraDevice();

    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            robot.run();
            telemetry.addLine(robot.driveTrain.getPoseEstimate() + "");
            telemetry.update();
        }

        RobotControl.lastFieldOrientedPos = robot.driveTrain.getPoseEstimate();

        robot.reset();
    }

}
