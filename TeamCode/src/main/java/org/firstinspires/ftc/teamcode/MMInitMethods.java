package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.MMLib.Examples.ElevatorExample.ExampleElevator;
import org.firstinspires.ftc.teamcode.MMLib.Examples.Subsystems.ShooterPID;

/**
 * this class should contain all ur init methods
 * <p>
 * for example, subsystems init and gamepad key bindings.
 * </p>
 * those methods should be called in the initTele(), initAuto() and initDebug() methods,
 * in the {@link MMRobot} class (you can also use those for experimenting).
 */
public class MMInitMethods {


    //For example:
    public static void initShooterPID() {
        MMRobot.getInstance().mmSystems.shooterPID = new ShooterPID();
    }

    public static void initExampleElevator() {
        MMRobot.getInstance().mmSystems.exampleElevator = new ExampleElevator();
    }

}
