package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.Side;
public class TeleOpMode extends CommandOpMode {
    RobotControl robot;

    AllianceColor allianceColor;

    private boolean firstIteration = true;

    public TeleOpMode(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    @Override
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.TELEOP, allianceColor, hardwareMap, gamepad1, gamepad2, telemetry);

        schedule(
                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.INIT)
        );
    }

    @Override
    public void run() {
        super.run();
        if(firstIteration) {
            robot.intake.lifter.setPosition(Intake.LifterPosition.DEFAULT);
            robot.droneLauncher.setState(DroneLauncher.State.HOLD);
            firstIteration = false;
        }

        ArmPositionSelector.telemetry(telemetry);

        telemetry.addData("selectedPosition", ArmPositionSelector.getPosition());
        telemetry.addData("isLeftOfBoard", ArmPositionSelector.getIsLeftOfBoard());
        telemetry.update();
    }
}