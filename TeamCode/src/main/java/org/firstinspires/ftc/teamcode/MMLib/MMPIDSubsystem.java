package org.firstinspires.ftc.teamcode.MMLib;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

public abstract class MMPIDSubsystem extends SubsystemBase {

    private final PIDController pidController;

    public MMPIDSubsystem(double kP, double kI, double kD, double tolerance) {
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
    }

    public PIDController getPidController() {
        return pidController;
    }

    public abstract void setPower(double power);

    public abstract double getCurrentValue();

    public void stop() {}

}
