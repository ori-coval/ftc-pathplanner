package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMEncoder;
import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMMotor;
import org.firstinspires.ftc.teamcode.MMLib.MMPIDSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;


@Config
public class ShooterPID extends MMPIDSubsystem {

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 0;

    private final MMMotor motor;
    private final MMEncoder encoder;

    private final MMRobot mmRobot = MMRobot.getInstance();

    public ShooterPID() {
        super(kP, kI, kD, tolerance);
        this.motor = mmRobot.mmSystems.controlHub.getMotor(Configuration.SHOOTER);
        this.encoder = mmRobot.mmSystems.controlHub.getEncoder(Configuration.SHOOTER, 3895.9);
    }

    @Override
    public void setPower(double power) {
        motor.setPower(power);
    }

    @Override
    public double getCurrentValue() {
        return encoder.getVelocity();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void periodic() {
        mmRobot.mmSystems.controlHub.pullBulkData();
        telemetry();
    }

    private void telemetry() {
        mmRobot.mmSystems.telemetry.addData("Current Value", getCurrentValue());
        FtcDashboard.getInstance().getTelemetry().addData("Current Value", getCurrentValue());
        FtcDashboard.getInstance().getTelemetry().addData("Target", getPidController().getSetPoint());
    }

}
