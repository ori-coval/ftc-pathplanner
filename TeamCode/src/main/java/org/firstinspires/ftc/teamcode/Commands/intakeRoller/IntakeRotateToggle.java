package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.CartridgeSetState;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetLifterPosition;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeRotateToggle extends CommandBase {
    public static boolean isRollerActive;
    RobotControl robot;
    public IntakeRotateToggle(RobotControl robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        new IntakeRotate(robot.intake.roller, isRollerActive ? 0 : robot.intake.roller.COLLECT_POWER).schedule();
        robot.schedule(intakeArmCommand(robot));
        isRollerActive = !isRollerActive;
    }

    public static Command intakeArmCommand(RobotControl robot) {
        return new ConditionalCommand(
                new SequentialCommandGroup( //turning off
                        new IntakeSetLifterPosition(robot.intake.lifter, Intake.LifterPosition.DEFAULT),
                        new CartridgeSetState(robot.cartridge, Cartridge.State.CLOSED_TWO_PIXELS),
                        new ArmGetToPosition(robot, ArmPosition.DRIVING_INTAKE, true)
                ),
                new SequentialCommandGroup( //turning on
                        new ArmGetToPosition(robot, ArmPosition.INTAKE, true),
                        new CartridgeSetState(robot.cartridge, Cartridge.State.INTAKE_OPEN)
                ),
                () -> isRollerActive
        );
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
