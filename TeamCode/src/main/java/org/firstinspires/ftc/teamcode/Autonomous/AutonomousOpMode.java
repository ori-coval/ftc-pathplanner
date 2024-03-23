package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.auto.AutoInit;
import org.firstinspires.ftc.teamcode.Commands.auto.LeaveSpikeMark;
import org.firstinspires.ftc.teamcode.Commands.auto.GoToStackForFirstCycleAndCollect;
import org.firstinspires.ftc.teamcode.Commands.auto.GoToStackForSecondCycleAndCollect;
import org.firstinspires.ftc.teamcode.Commands.auto.Parking;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoreYellowClose;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoreYellowFar;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringPurplePixel;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringFirstCycleAuto;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringSecondCycleAuto;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

public class AutonomousOpMode extends LinearOpMode {

    public static RobotControl robot;

    public AllianceColor allianceColor;

    public AllianceSide allianceSide;

    public AutonomousOpMode(AllianceColor allianceColor, AllianceSide allianceSide) {
        this.allianceColor = allianceColor;
        this.allianceSide = allianceSide;
    }

    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, allianceColor, allianceSide, hardwareMap, gamepad1, gamepad2, telemetry);
        SequentialCommandGroup commandsToRun = null;

        while(opModeInInit() && !isStopRequested()) {
            if(robot.teamPropDetector.getTeamPropSide() != null) {
                commandsToRun = getCommandsToRun();
            }
            robot.teamPropDetector.telemetry();
        }
        robot.schedule(commandsToRun);
        robot.teamPropDetector.webcam.closeCameraDevice();

    }

    private SequentialCommandGroup getCommandsToRun() {
        SequentialCommandGroup result = new SequentialCommandGroup(
                new WaitUntilCommand(this::isStarted),
                new AutoInit(),
                new ScoringPurplePixel()
        );
        if(allianceSide == AllianceSide.FAR) {
            result.addCommands(
                    new LeaveSpikeMark(),
                    new ScoreYellowFar(robot),
                    new GoToStackForFirstCycleAndCollect(),
                    new ScoringFirstCycleAuto(robot),
                    new GoToStackForSecondCycleAndCollect(),
                    new ScoringSecondCycleAuto(robot)
            );
        } else {
            result.addCommands(
                    new ScoreYellowClose()
            );
        }
        result.addCommands(new Parking());
        return result;

    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            robot.run();
            telemetry.addData("pixelCount", robot.intake.roller.getPixelCount());
            telemetry.addData("poseEstimateX", robot.autoDriveTrain.getPoseEstimate().getX());
            telemetry.addData("poseEstimateY", robot.autoDriveTrain.getPoseEstimate().getY());
            telemetry.update();
        }

        RobotControl.lastHeading = robot.driveTrain.getYawInDegrees();

        robot.reset();
    }

}
