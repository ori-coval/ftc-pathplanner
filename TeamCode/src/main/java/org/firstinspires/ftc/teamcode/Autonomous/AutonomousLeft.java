package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.auto.AutoInit;
import org.firstinspires.ftc.teamcode.Commands.auto.GoFromSpikeMarkToStackAndCollect;
import org.firstinspires.ftc.teamcode.Commands.auto.ParkingAfterScoringYellow;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringFirstPixelAuto;
import org.firstinspires.ftc.teamcode.Commands.auto.ScoringPurplePixel;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.Side;

public class AutonomousLeft extends CommandOpMode {
    RobotControl robot;
    AllianceColor allianceColor;
    public AutonomousLeft(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    @Override
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, allianceColor, Side.LEFT, hardwareMap, gamepad1, gamepad2, telemetry);

        while(opModeInInit()) {
            if(robot.teamPropDetector.getTeamPropSide() != null) {
                schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(), //for some reason it runs the first command on the init
                                new AutoInit(robot),
                                new ScoringPurplePixel(robot),
                                new GoFromSpikeMarkToStackAndCollect(robot),
                                new ScoringFirstPixelAuto(robot),
                                new ParkingAfterScoringYellow(robot)
                        )
                );
            }
            robot.teamPropDetector.telemetry();
        }

    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
