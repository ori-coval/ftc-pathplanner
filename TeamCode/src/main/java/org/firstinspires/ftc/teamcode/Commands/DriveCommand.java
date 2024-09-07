package org.firstinspires.ftc.teamcode.Commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

public class DriveCommand extends RunCommand {
    //honestly im doing those just to reduce code
    static MMSystems mmSystems = MMRobot.getInstance().mmSystems;
    static GamepadEx gamepadEx1 = mmSystems.gamepadEx1;

    public DriveCommand() {
        super(
                () -> mmSystems.mmDriveTrain.fieldOrientedDrive(
                        gamepadEx1.getLeftX(),
                        -gamepadEx1.getLeftY(),
                        gamepadEx1.getRightX()
                ),
                mmSystems.mmDriveTrain
        );
    }

    /**
     * Creates a new RunCommand.  The Runnable will be run continuously until the command
     * ends.  Does not run when disabled.
     *
     * @param toRun        the Runnable to run
     * @param requirements the subsystems to require
     */
    public DriveCommand(@NonNull Runnable toRun, Subsystem... requirements) {
        super(toRun, requirements);
    }
}
