package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

public class DriveCommand extends RunCommand {
    //honestly im doing those just to reduce code
    static MMSystems mmSystems = MMRobot.getInstance().mmSystems;
    static GamepadEx gamepadEx1 = mmSystems.gamepadEx1;

    public DriveCommand() {
        super(
                () -> mmSystems.driveTrain.fieldOrientedDrive(
                        gamepadEx1.getLeftX(),
                        -gamepadEx1.getLeftY(),
                        gamepadEx1.getRightX()
                ),
                mmSystems.driveTrain
        );
    }
}
