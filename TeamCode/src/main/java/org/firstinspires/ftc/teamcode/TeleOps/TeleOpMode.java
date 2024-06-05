package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

public abstract class TeleOpMode extends CommandOpMode {
    RobotControl robot; //your all-in-one subsystem storage solution
    AllianceColor allianceColor;
    private boolean isFirstIteration = true;

    public TeleOpMode(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    @Override
    public void initialize() {
        robot = new RobotControl(
                RobotControl.OpModeType.TELEOP,
                allianceColor,
                hardwareMap,
                gamepad1,
                gamepad2,
                telemetry
        );

        schedule(
                //get your robot to it's starting position
        );
    }

    @Override
    public void run() {
        super.run();
        if (isFirstIteration) {
            //do stuff that needs to happen after init
            isFirstIteration = false;
        }

        //telemetry here
        telemetry.update();
    }
}