package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.auto.AutoInit;
import org.firstinspires.ftc.teamcode.Commands.auto.GoFromSpikeMarkToStackAndCollect;
import org.firstinspires.ftc.teamcode.Commands.auto.ParkingAfterScoringYellow;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringFirstPixelAuto;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringPurplePixel;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

public class AutonomousFar extends LinearOpMode {
    RobotControl robot;
    AllianceColor allianceColor;
    public AutonomousFar(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, allianceColor, AllianceSide.FAR, hardwareMap, gamepad1, gamepad2, telemetry);
        SequentialCommandGroup commandsToRun = null;

        while(opModeInInit() && !isStopRequested()) {
            if(robot.teamPropDetector.getTeamPropSide() != null) {
                robot.teamPropDetector.webcam.closeCameraDevice();
                commandsToRun = new SequentialCommandGroup(
                        new WaitUntilCommand(this::isStarted),
                        new AutoInit(robot),
                        new ScoringPurplePixel(robot),
                        new GoFromSpikeMarkToStackAndCollect(robot),
                        new ScoringFirstPixelAuto(robot),
                        new ParkingAfterScoringYellow(robot)
                );

            }
            robot.teamPropDetector.telemetry();
        }
        robot.schedule(commandsToRun);

    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            robot.run();
            telemetry.update();
        }

        RobotControl.lastHeading = robot.driveTrain.getYawInDegrees();

        robot.reset();
    }
}
