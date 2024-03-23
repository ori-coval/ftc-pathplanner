package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetLifterPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class AutoInit extends SequentialCommandGroup {
    public AutoInit() {
        RobotControl robot = AutonomousOpMode.robot;
        addCommands(
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_ONE_PIXEL),
                new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT)
        );
    }
}
