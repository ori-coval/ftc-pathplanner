package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends CommandOpMode {
    RobotControl robot;

    @Override
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.TELEOP, hardwareMap, gamepad1, gamepad2, telemetry);

        schedule( //will happen on start
                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT)
        );
    }

    @Override
    public void run() {
        super.run();

        ArmPositionSelector.telemetry(telemetry);

        telemetry.addData("pixel Count", robot.intake.roller.getPixelCount());
        telemetry.addData("isRobotFull", robot.intake.roller.isRobotFull());
        telemetry.addData("selectedPosition", ArmPositionSelector.getPosition());
        telemetry.addData("isLeftOfBoard", ArmPositionSelector.getIsLeftOfBoard());
        telemetry.update();
    }
}