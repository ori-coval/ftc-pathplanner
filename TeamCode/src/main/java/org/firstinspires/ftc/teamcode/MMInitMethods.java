package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.SubSystems.ShooterPID;

/**
 * this class should contain all ur init methods
 * <p>
 * for example, subsystems init and gamepad key bindings.
 * </p>
 * those methods should be called in the initTele(), initAuto() and initDebug() methods,
 * in the {@link MMRobot} class (you can also use those for experimenting).
 */
public class MMInitMethods {

    public static void initShooterPID() {
        MMRobot.getInstance().mmSystems.shooterPID = new ShooterPID();
    }

}
