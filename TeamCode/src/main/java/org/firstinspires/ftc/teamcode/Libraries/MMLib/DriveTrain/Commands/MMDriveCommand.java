package org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Commands;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

/**
 * this class runs the
 * {@link org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Subsystem.MMDriveTrain#fieldOrientedDrive(double, double, double) fieldOrientedDrive()}
 * with {@link com.arcrobotics.ftclib.gamepad.GamepadEx gamepadEx1}.
 */
public class MMDriveCommand extends RunCommand {

    //honestly im doing those just to reduce code
    static MMSystems mmSystems = MMRobot.getInstance().mmSystems;
    static GamepadEx gamepadEx1 = mmSystems.gamepadEx1;

    public MMDriveCommand() {
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
