package org.firstinspires.ftc.teamcode.Commands.intakeRoller;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeRotateToggle extends ConditionalCommand {
    private static boolean rollerState = false;
    public IntakeRotateToggle(Intake.Roller intakeRoller) {
        super(
                new IntakeRotate(intakeRoller, 0),
                new IntakeRotate(intakeRoller, intakeRoller.COLLECT_POWER),
                () -> rollerState
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        rollerState = !rollerState;
    }
}
