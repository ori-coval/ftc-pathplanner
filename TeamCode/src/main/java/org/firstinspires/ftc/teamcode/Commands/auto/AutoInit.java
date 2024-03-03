package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class AutoInit extends SequentialCommandGroup {
    public AutoInit(RobotControl robot) {
        super(
                new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_ONE_PIXEL),
                new IntakeSetStackPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT)
        );
    }
}
