package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.ElevatorPIDExample.ExampleElevator;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems.ShooterPID;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.RollerIntake;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;

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
        mmSystems.driveTrain = new DriveTrain();
        mmSystems.driveTrain.setDefaultCommand(
                new DriveCommand()
        );
    }

    public static void initShooterPID() {
        mmSystems.shooterPID = new ShooterPID();
    }

    public static void initExampleElevator() {
        mmSystems.exampleElevator = new ExampleElevator();
    }

    public static void initArmAngle() {
        mmSystems.armAngle = new IntakeArm();
    }

    public static void initLinearIntake() {
        mmSystems.linearIntake = new LinearIntake();
    }
    public static void initIntake() {
        mmSystems.intake = new RollerIntake();
    }
    public static void initClaw() {
        mmSystems.claw = new Claw();
    }
    public static void initElevator() {
        mmSystems.elevator = new Elevator();
    }


}
