package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems.ShooterPID;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMTuningFFCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

import org.firstinspires.ftc.teamcode.Utils.OpModeType;

public class TuningFFTeleOp extends MMTeleOp {

    private final MMRobot mmRobot = MMRobot.getInstance();

    public TuningFFTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {

        mmRobot.mmSystems.shooterPID = new ShooterPID();

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new MMTuningFFCommand(mmRobot.mmSystems.shooterPID, 10, 0.002)
        );

    }

    @Override
    public void run() {
        super.run();
        mmRobot.mmSystems.controlHub.pullBulkData();
        telemetry.update();
    }
}
