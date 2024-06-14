package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Shooter extends SubsystemBase {

    DcMotorEx motor;

    public Shooter() {
        this.motor = MMRobot.getInstance().mmSystems.hardwareMap.get(DcMotorEx.class, Configuration.SHOOTER);
        this.register();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public boolean isActive() {
        return motor.getPower() != 0;
    }

    @Override
    public void periodic() {
        MMRobot.getInstance().mmSystems.telemetry.addLine(String.valueOf(motor.getPower()));
    }
}
