package org.firstinspires.ftc.teamcode.MMLib.PID;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.MMLib.Subsystems.MMPowerSubsystem;

public abstract class MMPIDSubsystem extends MMPowerSubsystem<Double> {

    private final PIDController pidController;

    public MMPIDSubsystem(double kP, double kI, double kD, double tolerance) {
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
    }

    public PIDController getPidController() {
        return pidController;
    }

    /**
     * this method should return the value to be fed into the PID controller
     * @return subsystem value
     */
    public abstract double getCurrentValue();

    /**
     * this method will help u add a FF controller.
     * @return the power to feed to the system
     */
    public double getFeedForwardPower() {
        return 0;
    }

    /**
     * this method will be called once the pidCommand isFinished (atSetPoint)
     */
    public void stop() {}

}
