package org.firstinspires.ftc.teamcode.MMLib.Examples.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMLib.PID.MMTuningFFCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

import org.firstinspires.ftc.teamcode.MMLib.Examples.Subsystems.ShooterPID;

public class TuningFFTeleOp extends MMTeleOp {

    private final MMRobot mmRobot = MMRobot.getInstance();

    public TuningFFTeleOp() {
        super(false);
    }

    @Override
    public void main() {

        mmRobot.mmSystems.shooterPID = new ShooterPID();

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new MMTuningFFCommand(mmRobot.mmSystems.shooterPID, 10, 0.002)
        );

    }
}
