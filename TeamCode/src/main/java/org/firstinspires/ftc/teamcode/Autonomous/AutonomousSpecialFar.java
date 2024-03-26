package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.auto.AutoInit;
import org.firstinspires.ftc.teamcode.Commands.auto.LeaveSpikeMark;
import org.firstinspires.ftc.teamcode.Commands.auto.GoToStackForFirstCycleAndCollect;
import org.firstinspires.ftc.teamcode.Commands.auto.Parking;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoreYellowClose;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoreYellowFar;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringPurplePixel;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringFirstCycleAuto;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;

public class AutonomousSpecialFar extends LinearOpMode {

    public static RobotControl robot;

    public AllianceColor allianceColor;


    public AutonomousSpecialFar(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, allianceColor, AllianceSide.FAR, hardwareMap, gamepad1, gamepad2, telemetry);
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
        return new SequentialCommandGroup(
                new WaitUntilCommand(this::isStarted),
                new AutoInit(),
                new ScoringPurplePixel(),
                new LeaveSpikeMark(),
                doOnlyIfClose(new WaitCommand(8000)),
                new ScoreYellowFar(robot),
                doOnlyIfNotClose(
                        new SequentialCommandGroup(
                            new GoToStackForFirstCycleAndCollect(),
                            new ScoringFirstCycleAuto(robot)
                        )
                ),
                new Parking()
        );
    }

    private Command doOnlyIfClose(Command commandToRun) {
        return new ConditionalCommand(
                commandToRun,
                new InstantCommand(),
                () -> robot.teamPropDetector.getTeamPropSide() == DetectionSide.CLOSE
        );
    }

    private Command doOnlyIfNotClose(Command commandToRun) {
        return new ConditionalCommand(
                commandToRun,
                new InstantCommand(),
                () -> robot.teamPropDetector.getTeamPropSide() != DetectionSide.CLOSE
        );
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            robot.run();
            FtcDashboard.getInstance().getTelemetry().addData("pixelCount", robot.intake.roller.getPixelCount());
            FtcDashboard.getInstance().getTelemetry().addData("isRobotFull", robot.intake.roller.isRobotFull());
            telemetry.addData("pixelCount", robot.intake.roller.getPixelCount());
            telemetry.addData("teamPropSide", robot.teamPropDetector.getTeamPropSide());
            telemetry.addData("poseEstimateX", robot.autoDriveTrain.getPoseEstimate().getX());
            telemetry.addData("poseEstimateY", robot.autoDriveTrain.getPoseEstimate().getY());
            FtcDashboard.getInstance().getTelemetry().update();
            telemetry.update();
        }

        RobotControl.lastHeading = robot.driveTrain.getYawInDegrees();

        robot.reset();
    }

}
