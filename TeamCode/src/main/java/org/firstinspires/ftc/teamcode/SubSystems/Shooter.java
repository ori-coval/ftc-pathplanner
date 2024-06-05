package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Shooter extends SubsystemBase {

    DcMotorEx motor;

    public Shooter(HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(DcMotorEx.class, Configuration.SHOOTER);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getVelocity() {
        return motor.getVelocity();
    }
}
