package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.CuttleMotor;
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
    private final CuttleEncoder encoder;

    private final MMRobot mmRobot = MMRobot.getInstance();

    public ShooterPID() {
        super(kP, kI, kD, tolerance);
        this.motor = mmRobot.mmSystems.controlHub.getMotor(Configuration.SHOOTER);
        this.encoder = mmRobot.mmSystems.controlHub.getEncoder(Configuration.SHOOTER, 3895);
    }

    @Override
    public void setPower(double power) {
        motor.setPower(power);
    }

    @Override
    public double getCurrentValue() {
        return encoder.getRotation();
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
