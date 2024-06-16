package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;

import org.firstinspires.ftc.teamcode.MMLib.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMLib.MMPIDSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class ShooterPID extends MMPIDSubsystem {

    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double tolerance = 0;

    private final CuttleMotor motor;

    private final MMRobot mmRobot = MMRobot.getInstance();

    public ShooterPID() {
        super(kP, kI, kD, tolerance);
        this.motor = MMRobot.getInstance().mmSystems.controlHub.getMotor(Configuration.SHOOTER);
    }

    @Override
    public void setPower(double power) {
        motor.setPower(power);
    }

    @Override
    public double getCurrentValue() {
        return motor.positionController.getEnc().getRotation();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void periodic() {
        mmRobot.mmSystems.telemetry.addData("Current Value", getCurrentValue());
    }
}
