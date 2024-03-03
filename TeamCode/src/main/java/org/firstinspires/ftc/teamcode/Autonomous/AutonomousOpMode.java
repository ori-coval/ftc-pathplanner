package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.auto.AutoInit;
import org.firstinspires.ftc.teamcode.Commands.auto.GoFromSpikeMarkToStackAndCollect;
import org.firstinspires.ftc.teamcode.Commands.auto.Parking;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoreYellowClose;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringFirstPixelAuto;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringPurplePixel;
import org.firstinspires.ftc.teamcode.Commands.auto.SecondCycle;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

public class AutonomousOpMode extends LinearOpMode {

    RobotControl robot;

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
                new AutoInit(robot),
                new ScoringPurplePixel(robot)
        );
        if(allianceSide == AllianceSide.FAR) {
            result.addCommands(
                    new GoFromSpikeMarkToStackAndCollect(robot),
                    new ScoringFirstPixelAuto(robot)/*,
                    new SecondCycle(robot)*/
            );
        } else {
            result.addCommands(
                    new ScoreYellowClose(robot)
            );
        }
        result.addCommands(new Parking(robot));
        return result;

    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            robot.run();
            telemetry.addData("poseEstimateX", robot.autoDriveTrain.getPoseEstimate().getX());
            telemetry.addData("poseEstimateY", robot.autoDriveTrain.getPoseEstimate().getY());
            telemetry.addData("pixelCount", robot.intake.roller.getPixelCount());
            telemetry.update();
        }

        RobotControl.lastHeading = robot.driveTrain.getYawInDegrees();

        robot.reset();
    }

}
