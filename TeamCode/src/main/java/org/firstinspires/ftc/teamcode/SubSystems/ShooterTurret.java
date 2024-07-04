package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMEncoder;
import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMMotor;
import org.firstinspires.ftc.teamcode.MMLib.PID.MMPIDSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class ShooterTurret extends MMPIDSubsystem {

    MMRobot mmRobot = MMRobot.getInstance();

    MMMotor motor;
    MMEncoder encoder;

    private final double GEAR_RATIO = (21./95);
    private final double TICKS_PER_REV = 8192;

    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double tolerance = 0;

    public ShooterTurret() {
        super(kP, kI, kD, tolerance);
        motor = mmRobot.mmSystems.controlHub.getMotor(Configuration.SHOOTER_TURRET);
        encoder = mmRobot.mmSystems.controlHub.getEncoder(Configuration.SHOOTER_TURRET, 0 /*idk*/);
    }


    @Override
    public double getCurrentValue() {
        return encoder.getCounts()/TICKS_PER_REV * 360 * GEAR_RATIO;
    }

    @Override
    public void setPower(Double power) {
        motor.setPower(power);
    }

    @Override
    public void periodic() {
        mmRobot.mmSystems.controlHub.pullBulkData();
    }
}
