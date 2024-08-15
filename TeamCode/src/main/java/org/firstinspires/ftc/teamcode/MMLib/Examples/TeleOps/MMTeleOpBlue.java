package org.firstinspires.ftc.teamcode.MMLib.Examples.TeleOps;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;

@TeleOp(name = "MMTeleOpBlue")
public class MMTeleOpBlue extends MMTeleOpTest {

    public MMTeleOpBlue() {
        super(
                AllianceColor.BLUE
        );
    }

    @Override
    public void main() {
        addRunnableOnInit(
                () -> MMRobot.getInstance().mmSystems.shooterPID.setPower(0.5)
        );

        addCommandsOnRun(
                new InstantCommand(
                        () -> MMRobot.getInstance().mmSystems.shooterPID.stop()
                )
        );
    }

}
