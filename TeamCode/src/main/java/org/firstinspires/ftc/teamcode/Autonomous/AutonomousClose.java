package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.auto.AutoInit;
import org.firstinspires.ftc.teamcode.Commands.auto.ParkingRight;
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

        while (opModeInInit()) {
            if(robot.teamPropDetector != null) {
                schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(),
                                new AutoInit(robot),
                                new ScoringPurplePixel(robot),
                                new ParkingRight(robot)
                        )
                );
            }
        }
        robot.teamPropDetector.telemetry();
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
