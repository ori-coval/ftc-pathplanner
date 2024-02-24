package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.auto.AutoInit;
import org.firstinspires.ftc.teamcode.Commands.auto.GoFromSpikeMarkToStackAndCollect;
import org.firstinspires.ftc.teamcode.Commands.auto.ParkingAfterScoringYellow;
import org.firstinspires.ftc.teamcode.Commands.auto.ParkingRight;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoreYellowClose;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringFirstPixelAuto;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringPurplePixel;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.Side;

public class AutonomousClose extends CommandOpMode {

    RobotControl robot;

    public AllianceColor allianceColor;

    public AutonomousClose(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    @Override
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

                schedule(commandsToRun);
            }
            robot.teamPropDetector.telemetry();
        }
        robot.teamPropDetector.webcam.closeCameraDevice();

    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
