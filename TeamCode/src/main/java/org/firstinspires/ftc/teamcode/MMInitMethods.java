package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.ElevatorPIDExample.ExampleElevator;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems.ShooterPID;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Commands.MMDriveCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Subsystem.MMDriveTrain;

/**
 * this class should contain all ur init methods
 * <p>
 * for example, subsystems init and gamepad key bindings.
 * </p>
 * those methods should be called in the initTele(), initAuto() and initDebug() methods,
 * in the {@link MMRobot} class (you can also use those for experimenting).
 */
public class MMInitMethods {

    static MMSystems mmSystems = MMRobot.getInstance().mmSystems; /*again just reducing code*/

    //For example:

    public static void initDriveTrain() {
        mmSystems.mmDriveTrain = new MMDriveTrain();
        mmSystems.mmDriveTrain.setDefaultCommand(
                new MMDriveCommand()
        );
    }

    public static void initShooterPID() {
        mmSystems.shooterPID = new ShooterPID();
    }

    public static void initExampleElevator() {
        mmSystems.exampleElevator = new ExampleElevator();
    }

}
