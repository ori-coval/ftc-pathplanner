package org.firstinspires.ftc.teamcode.MMLib;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public abstract class MMPIDSubsystem extends SubsystemBase {

    public final double kP;
    public final double kI;
    public final double kD;
    public final double tolerance;

    private final PIDController pidController;

    public MMPIDSubsystem(double kP, double kI, double kD, double tolerance) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.tolerance = tolerance;
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
